// native/tests/gpu/test_gpu_sp4_closed_loop.cpp — C++23 verification for SP4-B
// closed-loop 4-pass M2_K2 GPU step integration:
// 1. Struct sizeof parity (Host vs Apple Metal)
// 2. Host FP32 vs Apple Metal 0-bit parity on pass0..3 + post_solve + 1-step nworld=1 vs 32 isolation
// 3. Short closed-loop rollout (400 steps = 0.5 s) verifying active leg actuation -> positive
//    delivered_crank_torque_nm -> positive Bosch CX assist & forward travel.

#include "../../src/configuration/load.hpp"
#include "../../src/golden/step_closure_bundle.hpp"
#include "../../src/gpu/cuda_kernels_abi.hpp"
#include "../../src/gpu/model_assets.hpp"
#include "../../src/gpu/runner_spindle_setup.hpp"
#include "../../src/gpu/runner_tyre_setup.hpp"
#include "../../src/gpu/sp4_closed_loop_step.hpp"
#include "../../src/model_access.hpp"
#include "../../src/terrain/trackfile.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

extern "C" {
#if defined(__APPLE__)
int bike_metal_init(const char *gpu_src_dir);
const char *bike_metal_device_name();

int bike_metal_sizeof_sp4_world_state();
int bike_metal_sizeof_spindle_params();

int bike_metal_launch_sp4_init(
    int nworld,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::gpu::GpuSp3Config *d_sp3_configs,
    int per_world_config,
    bike::gpu::GpuSp3WorldState *d_sp3_states,
    bike::gpu::GpuSp4WorldState *d_sp4_states,
    bike::gpu::GpuWorldState *d_world_states,
    const float *d_qpos,
    float init_grade,
    int ntendon,
    float *d_tendon_range,
    int tendon_range_world_stride
);

int bike_metal_launch_sp4_pre_solve_and_pass0(
    int nworld,
    float dt_s,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::physics_core::GpuTyreModel *d_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *d_terrain_desc,
    const float *d_terrain_f32,
    const bike::physics_core::i32 *d_terrain_i32,
    const bike::gpu::GpuSp3Config *d_sp3_configs,
    int per_world_config,
    const bike::physics_core::SpindleM2K2Params<float> *d_spindle_params,
    bike::gpu::GpuSp3WorldState *d_sp3_states,
    bike::gpu::GpuSp4WorldState *d_sp4_states,
    bike::gpu::GpuWorldState *d_world_states,
    const float *d_qpos,
    const float *d_qvel,
    const float *d_qfrc_bias,
    const float *d_xpos,
    const float *d_xmat,
    const float *d_xanchor,
    const float *d_xaxis,
    const float *d_site_xpos,
    const float *d_subtree_com,
    const float *d_body_subtreemass,
    int qM_row_stride,
    int M_world_stride,
    const int *d_M_rowadr,
    const int *d_M_rownnz,
    const int *d_M_colind,
    const float *d_M_or_qM,
    float *d_qfrc_applied,
    float *d_ctrl,
    int ntendon,
    float *d_tendon_range,
    int tendon_range_world_stride,
    float *d_dof_frictionloss,
    int dof_frictionloss_world_stride,
    float *d_qacc_warmstart = nullptr
);

int bike_metal_launch_sp4_pass1(
    int nworld, int nq, int nv, int nu,
    const bike::physics_core::SpindleM2K2Params<float> *d_spindle_params,
    bike::gpu::GpuSp4WorldState *d_sp4_states,
    const float *d_qpos, const float *d_qvel, const float *d_qfrc_bias, const float *d_qacc,
    int qM_row_stride, int M_world_stride,
    const int *d_M_rowadr, const int *d_M_rownnz, const int *d_M_colind,
    const float *d_M_or_qM, float *d_ctrl, float *d_qacc_warmstart = nullptr
);

int bike_metal_launch_sp4_pass2(
    int nworld, int nq, int nv, int nu,
    const bike::physics_core::SpindleM2K2Params<float> *d_spindle_params,
    bike::gpu::GpuSp4WorldState *d_sp4_states,
    const float *d_qpos, const float *d_qvel, const float *d_qfrc_bias, const float *d_qacc,
    int qM_row_stride, int M_world_stride,
    const int *d_M_rowadr, const int *d_M_rownnz, const int *d_M_colind,
    const float *d_M_or_qM, float *d_ctrl, float *d_qacc_warmstart = nullptr
);

int bike_metal_launch_sp4_pass3(
    int nworld, int nq, int nv, int nu,
    const bike::physics_core::SpindleM2K2Params<float> *d_spindle_params,
    bike::gpu::GpuSp4WorldState *d_sp4_states,
    const float *d_qpos, const float *d_qvel, const float *d_qfrc_bias, const float *d_qacc,
    int qM_row_stride, int M_world_stride,
    const int *d_M_rowadr, const int *d_M_rownnz, const int *d_M_colind,
    const float *d_M_or_qM, float *d_ctrl, float *d_qacc_warmstart = nullptr
);

int bike_metal_launch_sp4_post_solve(
    int nworld,
    float dt_s,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::gpu::GpuSp3Config *d_sp3_configs,
    int per_world_config,
    const bike::physics_core::SpindleM2K2Params<float> *d_spindle_params,
    bike::gpu::GpuSp3WorldState *d_sp3_states,
    bike::gpu::GpuSp4WorldState *d_sp4_states,
    float *d_qpos,
    const float *d_qvel,
    const float *d_xpos,
    const float *d_xmat,
    const float *d_xanchor,
    int nsensordata,
    const float *d_sensordata,
    const int *d_nefc,
    int njmax,
    int efc_J_row_stride,
    const int *d_efc_type,
    const int *d_efc_id,
    const float *d_efc_J,
    const float *d_efc_force,
    const int *d_nacon,
    int nconmax,
    int contact_efc_stride,
    const int *d_contact_worldid,
    const int *d_contact_geom,
    const int *d_contact_efc_address,
    const int *d_contact_dim
);

int bike_metal_launch_sp4_m0_init(
    int nworld,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::gpu::GpuSp3Config *d_sp3_configs,
    int per_world_config,
    bike::gpu::GpuSp3WorldState *d_sp3_states,
    bike::gpu::GpuSp4WorldState *d_sp4_states,
    bike::physics_core::SpindleM0Scratch<float> *d_m0_scratch,
    bike::gpu::GpuWorldState *d_world_states,
    const float *d_qpos,
    float init_grade,
    int ntendon,
    float *d_tendon_range,
    int tendon_range_world_stride
);

int bike_metal_launch_sp4_m0_pre_solve_and_begin(
    int nworld,
    float dt_s,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::physics_core::GpuTyreModel *d_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *d_terrain_desc,
    const float *d_terrain_f32,
    const bike::physics_core::i32 *d_terrain_i32,
    const bike::gpu::GpuSp3Config *d_sp3_configs,
    int per_world_config,
    const bike::physics_core::SpindleM2K2Params<float> *d_spindle_params,
    bike::gpu::GpuSp3WorldState *d_sp3_states,
    bike::gpu::GpuSp4WorldState *d_sp4_states,
    bike::physics_core::SpindleM0Scratch<float> *d_m0_scratch,
    bike::gpu::GpuWorldState *d_world_states,
    const float *d_qpos,
    const float *d_qvel,
    const float *d_qfrc_bias,
    const float *d_xpos,
    const float *d_xmat,
    const float *d_xanchor,
    const float *d_xaxis,
    const float *d_site_xpos,
    const float *d_subtree_com,
    const float *d_body_subtreemass,
    int qM_row_stride,
    int M_world_stride,
    const int *d_M_rowadr,
    const int *d_M_rownnz,
    const int *d_M_colind,
    const float *d_M_or_qM,
    float *d_qfrc_applied,
    float *d_ctrl,
    int ntendon,
    float *d_tendon_range,
    int tendon_range_world_stride,
    float *d_dof_frictionloss,
    int dof_frictionloss_world_stride,
    float *d_qacc_warmstart,
    int *out_any_needs_solve
);

int bike_metal_launch_sp4_m0_step(
    int nworld,
    int nq,
    int nv,
    int nu,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::gpu::GpuSp3Config *d_sp3_configs,
    int per_world_config,
    const bike::gpu::GpuSp3WorldState *d_sp3_states,
    const bike::physics_core::SpindleM2K2Params<float> *d_spindle_params,
    bike::gpu::GpuSp4WorldState *d_sp4_states,
    bike::physics_core::SpindleM0Scratch<float> *d_m0_scratch,
    const float *d_qpos,
    const float *d_qvel,
    const float *d_qfrc_bias,
    const float *d_qacc,
    int qM_row_stride,
    int M_world_stride,
    const int *d_M_rowadr,
    const int *d_M_rownnz,
    const int *d_M_colind,
    const float *d_M_or_qM,
    float *d_ctrl,
    float *d_qacc_warmstart,
    int *out_any_needs_solve
);

int bike_metal_launch_sp4_m0_post_solve(
    int nworld,
    float dt_s,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::gpu::GpuSp3Config *d_sp3_configs,
    int per_world_config,
    const bike::physics_core::SpindleM2K2Params<float> *d_spindle_params,
    bike::gpu::GpuSp3WorldState *d_sp3_states,
    bike::gpu::GpuSp4WorldState *d_sp4_states,
    const bike::physics_core::SpindleM0Scratch<float> *d_m0_scratch,
    float *d_qpos,
    const float *d_qvel,
    const float *d_xpos,
    const float *d_xmat,
    const float *d_xanchor,
    int nsensordata,
    const float *d_sensordata,
    const int *d_nefc,
    int njmax,
    int efc_J_row_stride,
    const int *d_efc_type,
    const int *d_efc_id,
    const float *d_efc_J,
    const float *d_efc_force,
    const int *d_nacon,
    int nconmax,
    int contact_efc_stride,
    const int *d_contact_worldid,
    const int *d_contact_geom,
    const int *d_contact_efc_address,
    const int *d_contact_dim
);

int bike_metal_launch_sp4_energy_post_step(
    int nworld,
    float dt_s,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::physics_core::GpuTyreModel *d_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *d_terrain_desc,
    const float *d_terrain_f32,
    const bike::gpu::GpuSp3Config *d_sp3_configs,
    int per_world_config,
    const bike::physics_core::SpindleM2K2Params<float> *d_spindle_params,
    bike::gpu::GpuSp3WorldState *d_sp3_states,
    bike::gpu::GpuSp4WorldState *d_sp4_states,
    const bike::gpu::GpuWorldState *d_world_states,
    int qM_row_stride,
    int M_world_stride,
    const int *d_M_rowadr,
    const int *d_M_rownnz,
    const int *d_M_colind,
    const float *d_M_or_qM,
    const float *d_qpos,
    const float *d_qvel,
    const float *d_ctrl,
    const float *d_body_mass,
    const float *d_xipos,
    const float *d_gravity,
    const int *d_nefc,
    int njmax,
    int efc_J_row_stride,
    const int *d_efc_type,
    const float *d_efc_J,
    const float *d_efc_force
);
#endif
} // extern "C"

namespace {

namespace fs = std::filesystem;
namespace ma = ::model_access;

using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPtr = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

constexpr float kDtS = 0.00125f;

std::size_t to_index(long long v) {
    if (v < 0) {
        throw std::out_of_range("negative index");
    }
    return static_cast<std::size_t>(v);
}

int to_int(mjtSize v) {
    return static_cast<int>(v);
}

void require(bool cond, std::string_view msg) {
    if (!cond) {
        throw std::runtime_error(std::string(msg));
    }
}

void require_bits_f32(float actual, float expected, std::string_view label) {
    const std::uint32_t a = std::bit_cast<std::uint32_t>(actual);
    const std::uint32_t e = std::bit_cast<std::uint32_t>(expected);
    if (a != e) {
        throw std::runtime_error(std::format(
            "{}: 0-bit mismatch actual={} (0x{:08x}) expected={} (0x{:08x})",
            label, actual, a, expected, e));
    }
}

void require_ulp_f32(
    float actual,
    float expected,
    std::uint32_t max_ulp,
    std::string_view label
) {
    if (actual == 0.0f && expected == 0.0f) {
        return;
    }
    const std::int32_t a = std::bit_cast<std::int32_t>(actual);
    const std::int32_t e = std::bit_cast<std::int32_t>(expected);
    const bool same_sign = ((a < 0) == (e < 0));
    const std::uint32_t ulp = same_sign
        ? static_cast<std::uint32_t>(std::abs(a - e))
        : std::numeric_limits<std::uint32_t>::max();
    if (ulp > max_ulp && std::abs(actual - expected) > 1e-6f) {
        throw std::runtime_error(std::format(
            "{}: ULP mismatch actual={} expected={} ulp={} max_ulp={}",
            label, actual, expected, ulp, max_ulp));
    }
}

fs::path repo_root() {
    return {BIKE_REPO_ROOT};
}

std::string read_text(const fs::path &p) {
    std::ifstream ifs(p);
    require(ifs.is_open(), "failed to open file: " + p.string());
    return {std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>()};
}

void write_text(const fs::path &p, std::string_view content) {
    std::ofstream ofs(p, std::ios::binary);
    require(ofs.is_open(), "failed to open file for write: " + p.string());
    ofs.write(content.data(), static_cast<std::streamsize>(content.size()));
    require(ofs.good(), "failed writing file: " + p.string());
}

ModelPtr load_xml(const fs::path &xml_path) {
    std::array<char, 1024> mj_err{};
    mjModel *raw = mj_loadXML(
        xml_path.string().c_str(), nullptr, mj_err.data(), static_cast<int>(mj_err.size()));
    require(raw != nullptr, std::string("mj_loadXML failed: ") + mj_err.data());
    return {raw, &mj_deleteModel};
}

void seed_flat_welded_road(const mjModel *m) {
    const int hf_id = mj_name2id(m, mjOBJ_HFIELD, "road");
    const int terrain_geom = mj_name2id(m, mjOBJ_GEOM, "terrain");
    require(hf_id >= 0 && terrain_geom >= 0, "welded.xml requires road hfield and terrain geom");
    const auto hfield_nrow = ma::readonly_buffer(m->hfield_nrow, m->nhfield, "hfield_nrow");
    const auto hfield_ncol = ma::readonly_buffer(m->hfield_ncol, m->nhfield, "hfield_ncol");
    const auto hfield_adr = ma::readonly_buffer(m->hfield_adr, m->nhfield, "hfield_adr");
    const auto hfield_size = ma::readonly_buffer(m->hfield_size, 4 * m->nhfield, "hfield_size");
    const auto geom_pos = ma::readonly_buffer(m->geom_pos, 3 * m->ngeom, "geom_pos");
    const int nrow = hfield_nrow[to_index(hf_id)];
    const int ncol = hfield_ncol[to_index(hf_id)];
    const int adr = hfield_adr[to_index(hf_id)];
    const double sz = static_cast<double>(hfield_size[4 * to_index(hf_id) + 2]);
    const double pos_z = static_cast<double>(geom_pos[3 * to_index(terrain_geom) + 2]);
    const float flat_norm = static_cast<float>((0.0 - pos_z) / sz);
    const auto hf = ma::mutable_buffer(m->hfield_data, m->nhfielddata, "hfield_data");
    for (int r = 0; r < nrow; ++r) {
        for (int c = 0; c < ncol; ++c) {
            hf[to_index(adr + r * ncol + c)] = flat_norm;
        }
    }
}

struct TempSp4Models {
    fs::path dir;
    fs::path xml13;
    fs::path strength_json;
    fs::path envelope_json;

    explicit TempSp4Models(const fs::path &root) {
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        dir = fs::temp_directory_path() / std::format("bike_sp4b_models_{}", stamp);
        fs::create_directories(dir);
        const fs::path welded_src = root / "tests" / "fixtures" / "model" / "welded.xml";
        strength_json = root / "tests" / "fixtures" / "config" / "rider_strength_reference.json";
        envelope_json = root / "tests" / "fixtures" / "config" / "rider_joint_envelope_anatomical.json";
        xml13 = dir / "model_12gears.xml";

        const std::string raw_xml = read_text(welded_src);
        ModelPtr m_base = load_xml(welded_src);
        seed_flat_welded_road(m_base.get());
        if (m_base->nhfielddata > 0) {
            const fs::path bin_path = dir / "hfield_data.bin";
            std::ofstream ofs(bin_path, std::ios::binary);
            require(ofs.is_open(), "open temp hfield_data.bin");
            const auto hf = ma::readonly_buffer(m_base->hfield_data, m_base->nhfielddata, "hfield_data");
            for (const float v : hf) {
                const double d = static_cast<double>(v);
                const auto bytes = std::bit_cast<std::array<char, sizeof(double)>>(d);
                ofs.write(bytes.data(), sizeof(double));
            }
            require(ofs.good(), "write temp hfield_data.bin");
        }

        bike::configuration::SimulationPhysicsConfig phys{};
        phys.drive.motor_clutch = (raw_xml.find("drive_shaft_spin") != std::string::npos);
        const std::string xml13_str = bike::golden::expand_model_12gears_xml(raw_xml, phys);
        write_text(xml13, xml13_str);
    }

    TempSp4Models(const TempSp4Models &) = delete;
    TempSp4Models &operator=(const TempSp4Models &) = delete;
    TempSp4Models(TempSp4Models &&) = delete;
    TempSp4Models &operator=(TempSp4Models &&) = delete;

    ~TempSp4Models() {
        if (!dir.empty()) {
            std::error_code ec;
            fs::remove_all(dir, ec);
        }
    }
};

struct FlatTerrainAndTyre {
    bike::physics_core::GpuTyreModel tyre{};
    bike::physics_core::GpuTerrainTableDesc terrain_desc{};
    std::vector<float> terrain_f32;
    std::vector<bike::physics_core::i32> terrain_i32;
};

std::unique_ptr<FlatTerrainAndTyre> build_tyre_terrain(
    const fs::path &xml_path,
    const bike::gpu::GpuModelIndices &idx
) {
    auto out = std::make_unique<FlatTerrainAndTyre>();
    int f32_count = 0;
    int i32_count = 0;
    require(
        bike::gpu::build_runner_tyre_and_terrain_buffers(
            xml_path.string().c_str(),
            static_cast<int>(bike::physics_core::TyreBackendKind::compliant_2d),
            0, &idx, &out->tyre, &out->terrain_desc,
            nullptr, &f32_count, nullptr, &i32_count) == 0 && f32_count > 0,
        "terrain size query failed");
    out->terrain_f32.resize(to_index(f32_count));
    out->terrain_i32.resize(to_index(std::max(1, i32_count)));
    require(
        bike::gpu::build_runner_tyre_and_terrain_buffers(
            xml_path.string().c_str(),
            static_cast<int>(bike::physics_core::TyreBackendKind::compliant_2d),
            0, &idx, &out->tyre, &out->terrain_desc,
            out->terrain_f32.data(), &f32_count,
            out->terrain_i32.data(), &i32_count) == 0,
        "terrain fill failed");
    return out;
}

std::unique_ptr<bike::gpu::GpuSp3Config> build_sp4_sp3_config(const mjModel *m) {
    auto cfg_ptr = std::make_unique<bike::gpu::GpuSp3Config>();
    auto &cfg = *cfg_ptr;
    bike::gpu::gpu_init_default_sp3_config(cfg);

    cfg.apply_direct_crank_torque = 0;
    cfg.arm_standstill_start = 1;
    cfg.seed_lean_from_launch = 1;

    const auto jnt_dofadr = ma::readonly_buffer(m->jnt_dofadr, m->njnt, "jnt_dofadr");
    const auto sensor_adr = ma::readonly_buffer(m->sensor_adr, m->nsensor, "sensor_adr");

    cfg.efc_scan.grip_left_eq_id = mj_name2id(m, mjOBJ_EQUALITY, "connect_grip_left");
    cfg.efc_scan.grip_right_eq_id = mj_name2id(m, mjOBJ_EQUALITY, "connect_grip_right");
    cfg.efc_scan.saddle_eq_id = mj_name2id(m, mjOBJ_EQUALITY, "connect_saddle");
    cfg.efc_scan.foot_front_eq_id = mj_name2id(m, mjOBJ_EQUALITY, "connect_foot_front");
    cfg.efc_scan.foot_rear_eq_id = mj_name2id(m, mjOBJ_EQUALITY, "connect_foot_rear");
    cfg.efc_scan.frame_body_id = mj_name2id(m, mjOBJ_BODY, "frame");
    cfg.efc_scan.crank_dof = jnt_dofadr[to_index(mj_name2id(m, mjOBJ_JOINT, "crank_spin"))];
    cfg.efc_scan.front_wheel_dof = jnt_dofadr[to_index(mj_name2id(m, mjOBJ_JOINT, "front_wheel_spin"))];
    cfg.efc_scan.rear_wheel_dof = jnt_dofadr[to_index(mj_name2id(m, mjOBJ_JOINT, "rear_wheel_spin"))];
    int hbar_gid = mj_name2id(m, mjOBJ_GEOM, "geom_handlebar");
    if (hbar_gid < 0) {
        hbar_gid = mj_name2id(m, mjOBJ_GEOM, "handlebar");
    }
    cfg.efc_scan.handlebar_geom_id = hbar_gid;
    cfg.efc_scan.catch_geom_id = mj_name2id(m, mjOBJ_GEOM, "catch_plane");
    cfg.efc_scan.terrain_geom_count = 0;
    const int terrain_gid = mj_name2id(m, mjOBJ_GEOM, "terrain");
    if (terrain_gid >= 0) {
        std::span{cfg.efc_scan.terrain_geom_ids}[to_index(cfg.efc_scan.terrain_geom_count++)] =
            terrain_gid;
    }
    if (cfg.efc_scan.catch_geom_id >= 0 &&
        cfg.efc_scan.terrain_geom_count < bike::physics_core::kMaxTerrainGeoms) {
        std::span{cfg.efc_scan.terrain_geom_ids}[to_index(cfg.efc_scan.terrain_geom_count++)] =
            cfg.efc_scan.catch_geom_id;
    }
    cfg.efc_scan.rider_geom_count = 0;
    for (int g = 0; g < m->ngeom; ++g) {
        const char *gname = mj_id2name(m, mjOBJ_GEOM, g);
        if (gname != nullptr && std::string_view(gname).starts_with("geom_rider_") &&
            cfg.efc_scan.rider_geom_count < bike::physics_core::kMaxRiderCrashGeoms) {
            std::span{cfg.efc_scan.rider_geom_ids}[to_index(cfg.efc_scan.rider_geom_count++)] = g;
        }
    }

    const int motor_act = mj_name2id(m, mjOBJ_ACTUATOR, "mid_drive");
    cfg.motor_actuator_idx = motor_act;
    if (motor_act >= 0) {
        const auto ctrlrange = ma::readonly_buffer(m->actuator_ctrlrange, 2 * m->nu, "actuator_ctrlrange");
        cfg.motor_ctrl_min_nm = static_cast<float>(ctrlrange[2 * to_index(motor_act) + 0]);
        cfg.motor_ctrl_max_nm = static_cast<float>(ctrlrange[2 * to_index(motor_act) + 1]);
    }
    const int accel_id = mj_name2id(m, mjOBJ_SENSOR, "sensor_frame_accel");
    const int gyro_id = mj_name2id(m, mjOBJ_SENSOR, "sensor_frame_gyro");
    cfg.accel_sensor_adr = (accel_id >= 0) ? sensor_adr[to_index(accel_id)] : -1;
    cfg.gyro_sensor_adr = (gyro_id >= 0) ? sensor_adr[to_index(gyro_id)] : -1;

    const std::span<int> tid_span{cfg.shifting.cassette_tendon_ids};
    for (int c = 0; c < bike::physics_core::kCassetteCogCount; ++c) {
        const int tid = mj_name2id(m, mjOBJ_TENDON, std::format("freehub_cog_{:02d}", c).c_str());
        require(tid >= 0, "missing cassette tendon");
        tid_span[to_index(c)] = tid;
    }
    cfg.efc_scan.freehub_tendon_id = tid_span[0];
    cfg.efc_scan.crank_clutch_tendon_id = mj_name2id(m, mjOBJ_TENDON, "crank_clutch");
    return cfg_ptr;
}

template <class DstT, class SrcT>
void copy_cast(const SrcT *src, std::size_t n, std::vector<DstT> &dst) {
    dst.resize(n);
    const auto src_span = ma::readonly_buffer(src, static_cast<mjtSize>(n), "src");
    for (std::size_t i = 0; i < n; ++i) {
        dst[i] = static_cast<DstT>(src_span[i]);
    }
}

void test_sizeof_parity() {
    require(
        bike_gpu_sizeof_sp4_world_state() == static_cast<int>(sizeof(bike::gpu::GpuSp4WorldState)),
        "host sizeof(GpuSp4WorldState) mismatch");
    require(
        bike_gpu_sizeof_spindle_params() ==
            static_cast<int>(sizeof(bike::physics_core::SpindleM2K2Params<float>)),
        "host sizeof(SpindleM2K2Params<float>) mismatch");
#if defined(__APPLE__)
    require(
        bike_metal_sizeof_sp4_world_state() == static_cast<int>(sizeof(bike::gpu::GpuSp4WorldState)),
        "metal sizeof(GpuSp4WorldState) mismatch");
    require(
        bike_metal_sizeof_spindle_params() ==
            static_cast<int>(sizeof(bike::physics_core::SpindleM2K2Params<float>)),
        "metal sizeof(SpindleM2K2Params<float>) mismatch");
#endif
    std::cout << "[test_sizeof_parity] GpuSp4WorldState=" << sizeof(bike::gpu::GpuSp4WorldState)
              << " SpindleM2K2Params=" << sizeof(bike::physics_core::SpindleM2K2Params<float>) << '\n';
}

void test_host_vs_metal_and_multiworld_isolation(const TempSp4Models &models) {
    ModelPtr m13 = load_xml(models.xml13);
    const auto idx = bike::gpu::assets::resolve_gpu_model_indices(m13.get());
    require(bike::gpu::gpu_validate_model_indices(idx, to_int(m13->ntendon)), "invalid model indices");

    const auto sp_params = std::make_unique<bike::physics_core::SpindleM2K2Params<float>>();
    require(
        bike::gpu::build_runner_spindle_params(
            models.xml13.string().c_str(),
            models.strength_json.string().c_str(),
            models.envelope_json.string().c_str(),
            1, sp_params.get()) == 0,
        "build_runner_spindle_params failed");

    const auto tt = build_tyre_terrain(models.xml13, idx);
    const auto base_cfg = build_sp4_sp3_config(m13.get());

    DataPtr d0(mj_makeData(m13.get()), &mj_deleteData);
    require(d0 != nullptr, "mj_makeData failed");
    {
        std::vector<double> qp;
        std::vector<double> qv;
        if (bike::gpu::assets::load_captured_initial_state(
                models.xml13.string().c_str(), idx.nq, idx.nv, qp, qv) ==
            bike::gpu::assets::AssetStatus::kOk) {
            std::ranges::copy(qp, ma::mutable_buffer(d0->qpos, m13->nq, "qpos").begin());
            std::ranges::copy(qv, ma::mutable_buffer(d0->qvel, m13->nv, "qvel").begin());
        }
    }
    {
        const int rider_rx_jnt = mj_name2id(m13.get(), mjOBJ_JOINT, "rider_root_x");
        const int rider_rz_jnt = mj_name2id(m13.get(), mjOBJ_JOINT, "rider_root_z");
        const int f_geom = bike::gpu::find_wheel_contact_geom(
            m13.get(), idx.front_wheel_body, "front_contact", "geom_front_contact");
        const int r_geom = bike::gpu::find_wheel_contact_geom(
            m13.get(), idx.rear_wheel_body, "rear_contact", "geom_rear_contact");
        require(f_geom >= 0 && r_geom >= 0, "wheel contact geoms required");
        const auto jnt_qposadr = ma::readonly_buffer(m13->jnt_qposadr, m13->njnt, "jnt_qposadr");
        const auto jnt_dofadr = ma::readonly_buffer(m13->jnt_dofadr, m13->njnt, "jnt_dofadr");
        const auto geom_size = ma::readonly_buffer(m13->geom_size, 3 * m13->ngeom, "geom_size");
        const int rider_rz_qpos = jnt_qposadr[to_index(rider_rz_jnt)];
        const auto d_qp = ma::mutable_buffer(d0->qpos, m13->nq, "qpos");
        const auto d_qv = ma::mutable_buffer(d0->qvel, m13->nv, "qvel");
        d_qp[to_index(idx.root_x_qpos)] = 2.0;
        d_qp[to_index(jnt_qposadr[to_index(rider_rx_jnt)])] = 2.0;
        mj_fwdPosition(m13.get(), d0.get());

        const auto d_xpos =
            ma::readonly_buffer(d0->xpos, 3LL * static_cast<mjtSize>(idx.nbody), "xpos");
        const double f_r = geom_size[3 * to_index(f_geom)];
        const double r_r = geom_size[3 * to_index(r_geom)];
        const double min_clear = std::min(
            d_xpos[3 * to_index(idx.front_wheel_body) + 2] - f_r,
            d_xpos[3 * to_index(idx.rear_wheel_body) + 2] - r_r);
        const double dz = -0.015 - min_clear;
        d_qp[to_index(idx.root_z_qpos)] += dz;
        d_qp[to_index(rider_rz_qpos)] += dz;
        d_qv[to_index(idx.root_x_dof)] = 2.0;
        d_qv[to_index(jnt_dofadr[to_index(rider_rx_jnt)])] = 2.0;
        d_qv[to_index(idx.crank_spin_dof)] = 6.0;
        d_qv[to_index(idx.front_wheel_spin_dof)] = 2.0 / f_r;
        d_qv[to_index(idx.rear_wheel_spin_dof)] = 2.0 / r_r;
    }
    mj_fwdPosition(m13.get(), d0.get());
    mj_fwdVelocity(m13.get(), d0.get());
    mj_fwdActuation(m13.get(), d0.get());
    mj_fwdAcceleration(m13.get(), d0.get());
    mj_fwdConstraint(m13.get(), d0.get());

    constexpr int kNworld = 32;
    const int nq = idx.nq;
    const int nv = idx.nv;
    const int nu = idx.nu;
    const int nbody = idx.nbody;
    const int njnt = to_int(m13->njnt);
    const int nsite = to_int(m13->nsite);
    const int ntendon = to_int(m13->ntendon);
    const int tr_stride = 2 * ntendon;
    const int nsd = to_int(m13->nsensordata);
    const int nM = to_int(m13->nM);
    const int njmax = std::max(1, d0->nefc);

    std::vector<float> p_qpos, p_qvel, p_bias, p_qacc, p_xpos, p_xmat, p_xa, p_xx, p_site, p_sc, p_M;
    std::vector<float> p_tr, p_fl, p_sd, p_eJ, p_ef;
    std::vector<int> p_et, p_eid, m_ra, m_rn, m_ci;
    copy_cast(d0->qpos, to_index(nq), p_qpos);
    copy_cast(d0->qvel, to_index(nv), p_qvel);
    copy_cast(d0->qfrc_bias, to_index(nv), p_bias);
    copy_cast(d0->qacc, to_index(nv), p_qacc);
    copy_cast(d0->xpos, to_index(nbody) * 3U, p_xpos);
    copy_cast(d0->xmat, to_index(nbody) * 9U, p_xmat);
    copy_cast(d0->xanchor, to_index(njnt) * 3U, p_xa);
    copy_cast(d0->xaxis, to_index(njnt) * 3U, p_xx);
    copy_cast(d0->site_xpos, to_index(nsite) * 3U, p_site);
    copy_cast(d0->subtree_com, to_index(nbody) * 3U, p_sc);
    copy_cast(d0->M, to_index(nM), p_M);
    copy_cast(m13->tendon_range, to_index(tr_stride), p_tr);
    copy_cast(m13->dof_frictionloss, to_index(nv), p_fl);
    copy_cast(d0->sensordata, to_index(nsd), p_sd);
    copy_cast(d0->efc_J, to_index(njmax) * to_index(nv), p_eJ);
    copy_cast(d0->efc_force, to_index(njmax), p_ef);
    copy_cast(d0->efc_type, to_index(njmax), p_et);
    copy_cast(d0->efc_id, to_index(njmax), p_eid);
    copy_cast(m13->M_rowadr, to_index(nv), m_ra);
    copy_cast(m13->M_rownnz, to_index(nv), m_rn);
    copy_cast(m13->M_colind, to_index(nM), m_ci);

    const std::size_t kNwU = to_index(kNworld);
    std::vector<bike::gpu::GpuSp3Config> cfgs(kNwU, *base_cfg);
    for (int w = 0; w < kNworld; ++w) {
        auto &c = cfgs[to_index(w)];
        c.tire_stiff_scale = 0.88f + 0.008f * static_cast<float>(w);
        c.suspension.params.air_spring.gauge_pressure_psi *= (0.90f + 0.006f * static_cast<float>(w));
        c.shifting.initial_rear_teeth =
            bike::configuration::kCassette12sTeeth[to_index(w % bike::physics_core::kCassetteCogCount)];
        bike::gpu::gpu_init_suspension_preloads(c.suspension);
    }

    const auto tile = [&](const auto &proto) {
        using ElemT = typename std::decay_t<decltype(proto)>::value_type;
        std::vector<ElemT> out(kNwU * proto.size());
        for (int w = 0; w < kNworld; ++w) {
            std::ranges::copy(proto, out.begin() + static_cast<std::ptrdiff_t>(to_index(w) * proto.size()));
        }
        return out;
    };

    auto qpos_b = tile(p_qpos);
    auto qvel_b = tile(p_qvel);
    for (int w = 0; w < kNworld; ++w) {
        qpos_b[to_index(w) * to_index(nq) + to_index(idx.crank_spin_qpos)] += 0.04f * static_cast<float>(w);
    }
    const auto bias_b = tile(p_bias);
    const auto qacc_b = tile(p_qacc);
    const auto xpos_b = tile(p_xpos);
    const auto xmat_b = tile(p_xmat);
    const auto xa_b = tile(p_xa);
    const auto xx_b = tile(p_xx);
    const auto site_b = tile(p_site);
    const auto sc_b = tile(p_sc);
    const auto M_b = tile(p_M);
    auto tr_b = tile(p_tr);
    auto fl_b = tile(p_fl);
    const auto sd_b = tile(p_sd);
    const auto eJ_b = tile(p_eJ);
    const auto ef_b = tile(p_ef);
    const auto et_b = tile(p_et);
    const auto eid_b = tile(p_eid);
    const std::vector<int> nefc_b(kNwU, d0->nefc);

    std::vector<bike::gpu::GpuSp3WorldState> h_sp3(kNwU);
    std::vector<bike::gpu::GpuSp4WorldState> h_sp4(kNwU);
    std::vector<bike::gpu::GpuWorldState> h_ws(kNwU);
    std::vector<float> h_qpos = qpos_b;
    std::vector<float> h_qfrc(kNwU * to_index(nv), 0.0f);
    std::vector<float> h_ctrl(kNwU * to_index(nu), 0.0f);
    std::vector<float> h_tr = tr_b;
    std::vector<float> h_fl = fl_b;
    std::vector<float> ws0_b(kNwU * to_index(nv), 0.0f);
    for (int w = 0; w < kNworld; ++w) {
        for (int i = 0; i < nv; ++i) {
            ws0_b[to_index(w) * to_index(nv) + to_index(i)] =
                0.25f * static_cast<float>(i + 1) + 0.01f * static_cast<float>(w);
        }
    }
    std::vector<float> h_ws_qacc = ws0_b;

    require(
        bike_gpu_launch_sp4_init(
            nullptr, kNworld, &idx, cfgs.data(), 1,
            h_sp3.data(), h_sp4.data(), h_ws.data(), h_qpos.data(),
            0.0f, ntendon, h_tr.data(), tr_stride) == 0,
        "host sp4 init failed");
    const int rc_p0 = bike_gpu_launch_sp4_pre_solve_and_pass0(
        nullptr, kNworld, kDtS, &idx, &tt->tyre, &tt->terrain_desc,
        tt->terrain_f32.data(), tt->terrain_i32.data(),
        cfgs.data(), 1, sp_params.get(), h_sp3.data(), h_sp4.data(), h_ws.data(),
        h_qpos.data(), qvel_b.data(), bias_b.data(), xpos_b.data(), xmat_b.data(),
        xa_b.data(), xx_b.data(), site_b.data(), sc_b.data(), nullptr,
        0, nM, m_ra.data(), m_rn.data(), m_ci.data(), M_b.data(),
        h_qfrc.data(), h_ctrl.data(), ntendon, h_tr.data(), tr_stride, h_fl.data(), nv,
        h_ws_qacc.data());
    if (rc_p0 != 0) {
        const auto diag_sp3 = std::make_unique<bike::gpu::GpuSp3WorldState>(h_sp3[0]);
        const auto diag_sp4 = std::make_unique<bike::gpu::GpuSp4WorldState>(h_sp4[0]);
        const auto diag_ws = std::make_unique<bike::gpu::GpuWorldState>(h_ws[0]);
        auto diag_qf = h_qfrc;
        auto diag_ct = h_ctrl;
        auto diag_tr = tr_b;
        auto diag_fl = fl_b;
        const auto diag_st = bike::gpu::gpu_sp4_pre_solve_and_pass0_world(
            0, kDtS, idx, &tt->tyre, tt->terrain_desc,
            tt->terrain_f32.data(), tt->terrain_i32.data(),
            &cfgs[0], sp_params.get(), diag_sp3.get(), diag_sp4.get(),
            &diag_ws->compliant_wheels[0], &diag_ws->distributed_wheels[0][0],
            qpos_b.data(), qvel_b.data(), bias_b.data(), xpos_b.data(), xmat_b.data(),
            xa_b.data(), xx_b.data(), site_b.data(), sc_b.data(), nullptr,
            0, m_ra.data(), m_rn.data(), m_ci.data(), M_b.data(),
            diag_qf.data(), diag_ct.data(), ntendon, diag_tr.data(), diag_fl.data(),
            static_cast<float *>(nullptr));
        bike::physics_core::GpuTyreStepOutput ty_out{};
        bike::physics_core::ErrCode ty_err = bike::physics_core::ErrCode::none;
        const auto ty_st = bike::physics_core::gpu_step_both_wheels_tyre(
            &tt->tyre, tt->terrain_desc, tt->terrain_f32.data(), tt->terrain_i32.data(),
            cfgs[0].tire_stiff_scale, kDtS,
            xpos_b.data(), xmat_b.data(), xa_b.data(), xx_b.data(), qvel_b.data(),
            &diag_ws->compliant_wheels[0], &diag_ws->distributed_wheels[0][0],
            diag_qf.data(), ty_out, ty_err, false);
        require(false, std::format(
            "host sp4 pass0 failed rc={} diag_st={} ty_st={} ty_err={} fw_x={} fw_z={} rw_x={} rw_z={} pl={}",
            rc_p0, static_cast<int>(diag_st), static_cast<int>(ty_st), static_cast<int>(ty_err),
            xpos_b[3U * to_index(idx.front_wheel_body) + 0U],
            xpos_b[3U * to_index(idx.front_wheel_body) + 2U],
            xpos_b[3U * to_index(idx.rear_wheel_body) + 0U],
            xpos_b[3U * to_index(idx.rear_wheel_body) + 2U],
            tt->terrain_desc.polyline_vertex_count));
    }
    const std::vector<float> h_ctrl_p0 = h_ctrl;
    std::ranges::fill(h_ws_qacc, -999.0f);
    require(
        bike_gpu_launch_sp4_pass1(
            nullptr, kNworld, nq, nv, nu, sp_params.get(), h_sp4.data(),
            h_qpos.data(), qvel_b.data(), bias_b.data(), qacc_b.data(),
            0, nM, m_ra.data(), m_rn.data(), m_ci.data(), M_b.data(), h_ctrl.data(),
            h_ws_qacc.data()) == 0,
        "host sp4 pass1 failed");
    require(h_ws_qacc == ws0_b, "host sp4 pass1 warmstart restore mismatch");
    const std::vector<float> h_ctrl_p1 = h_ctrl;
    std::ranges::fill(h_ws_qacc, -777.0f);
    require(
        bike_gpu_launch_sp4_pass2(
            nullptr, kNworld, nq, nv, nu, sp_params.get(), h_sp4.data(),
            h_qpos.data(), qvel_b.data(), bias_b.data(), qacc_b.data(),
            0, nM, m_ra.data(), m_rn.data(), m_ci.data(), M_b.data(), h_ctrl.data(),
            h_ws_qacc.data()) == 0,
        "host sp4 pass2 failed");
    require(h_ws_qacc == ws0_b, "host sp4 pass2 warmstart restore mismatch");
    const std::vector<float> h_ctrl_p2 = h_ctrl;
    std::ranges::fill(h_ws_qacc, -555.0f);
    require(
        bike_gpu_launch_sp4_pass3(
            nullptr, kNworld, nq, nv, nu, sp_params.get(), h_sp4.data(),
            h_qpos.data(), qvel_b.data(), bias_b.data(), qacc_b.data(),
            0, nM, m_ra.data(), m_rn.data(), m_ci.data(), M_b.data(), h_ctrl.data(),
            h_ws_qacc.data()) == 0,
        "host sp4 pass3 failed");
    require(h_ws_qacc == ws0_b, "host sp4 pass3 warmstart restore mismatch");
    require(
        bike_gpu_launch_sp4_post_solve(
            nullptr, kNworld, kDtS, &idx, cfgs.data(), 1, sp_params.get(),
            h_sp3.data(), h_sp4.data(), h_qpos.data(), qvel_b.data(),
            xpos_b.data(), xmat_b.data(), xa_b.data(),
            nsd, sd_b.data(), nefc_b.data(), njmax, nv,
            et_b.data(), eid_b.data(), eJ_b.data(), ef_b.data(),
            nullptr, 0, 1, nullptr, nullptr, nullptr, nullptr) == 0,
        "host sp4 post_solve failed");

    // Host 1-step multi-world isolation (nworld=1 vs nworld=32) on worlds {0, 17, 31}
    for (const int w : {0, 17, 31}) {
        const std::size_t wu = to_index(w);
        auto s_sp3 = std::make_unique<bike::gpu::GpuSp3WorldState>();
        auto s_sp4 = std::make_unique<bike::gpu::GpuSp4WorldState>();
        const auto s_ws = std::make_unique<bike::gpu::GpuWorldState>();
        std::vector<float> s_qp(qpos_b.begin() + static_cast<std::ptrdiff_t>(wu * to_index(nq)),
                                qpos_b.begin() + static_cast<std::ptrdiff_t>((wu + 1U) * to_index(nq)));
        std::vector<float> s_qf(to_index(nv), 0.0f);
        std::vector<float> s_ct(to_index(nu), 0.0f);
        std::vector<float> s_tr(tr_b.begin() + static_cast<std::ptrdiff_t>(wu * to_index(tr_stride)),
                                tr_b.begin() + static_cast<std::ptrdiff_t>((wu + 1U) * to_index(tr_stride)));
        std::vector<float> s_fl(fl_b.begin() + static_cast<std::ptrdiff_t>(wu * to_index(nv)),
                                fl_b.begin() + static_cast<std::ptrdiff_t>((wu + 1U) * to_index(nv)));

        require(
            bike_gpu_launch_sp4_init(
                nullptr, 1, &idx, &cfgs[wu], 0, s_sp3.get(), s_sp4.get(), s_ws.get(),
                s_qp.data(), 0.0f, ntendon, s_tr.data(), tr_stride) == 0,
            "solo sp4 init failed");
        require(
            bike_gpu_launch_sp4_pre_solve_and_pass0(
                nullptr, 1, kDtS, &idx, &tt->tyre, &tt->terrain_desc,
                tt->terrain_f32.data(), tt->terrain_i32.data(),
                &cfgs[wu], 0, sp_params.get(), s_sp3.get(), s_sp4.get(), s_ws.get(),
                s_qp.data(), p_qvel.data(), p_bias.data(), p_xpos.data(), p_xmat.data(),
                p_xa.data(), p_xx.data(), p_site.data(), p_sc.data(), nullptr,
                0, nM, m_ra.data(), m_rn.data(), m_ci.data(), p_M.data(),
                s_qf.data(), s_ct.data(), ntendon, s_tr.data(), tr_stride, s_fl.data(), nv) == 0,
            "solo sp4 pass0 failed");
        require(
            bike_gpu_launch_sp4_pass1(
                nullptr, 1, nq, nv, nu, sp_params.get(), s_sp4.get(),
                s_qp.data(), p_qvel.data(), p_bias.data(), p_qacc.data(),
                0, nM, m_ra.data(), m_rn.data(), m_ci.data(), p_M.data(), s_ct.data()) == 0,
            "solo sp4 pass1 failed");
        require(
            bike_gpu_launch_sp4_pass2(
                nullptr, 1, nq, nv, nu, sp_params.get(), s_sp4.get(),
                s_qp.data(), p_qvel.data(), p_bias.data(), p_qacc.data(),
                0, nM, m_ra.data(), m_rn.data(), m_ci.data(), p_M.data(), s_ct.data()) == 0,
            "solo sp4 pass2 failed");
        require(
            bike_gpu_launch_sp4_pass3(
                nullptr, 1, nq, nv, nu, sp_params.get(), s_sp4.get(),
                s_qp.data(), p_qvel.data(), p_bias.data(), p_qacc.data(),
                0, nM, m_ra.data(), m_rn.data(), m_ci.data(), p_M.data(), s_ct.data()) == 0,
            "solo sp4 pass3 failed");
        require(
            bike_gpu_launch_sp4_post_solve(
                nullptr, 1, kDtS, &idx, &cfgs[wu], 0, sp_params.get(),
                s_sp3.get(), s_sp4.get(), s_qp.data(), p_qvel.data(),
                p_xpos.data(), p_xmat.data(), p_xa.data(),
                nsd, p_sd.data(), &d0->nefc, njmax, nv,
                p_et.data(), p_eid.data(), p_eJ.data(), p_ef.data(),
                nullptr, 0, 1, nullptr, nullptr, nullptr, nullptr) == 0,
            "solo sp4 post_solve failed");

        for (int i = 0; i < nv; ++i) {
            require_bits_f32(
                h_qfrc[wu * to_index(nv) + to_index(i)], s_qf[to_index(i)], "isolation qfrc");
        }
        for (int i = 0; i < nu; ++i) {
            require_bits_f32(
                h_ctrl[wu * to_index(nu) + to_index(i)], s_ct[to_index(i)], "isolation ctrl");
        }
        const std::span<const float> h_ht{h_sp4[wu].spindle.held_torques};
        const std::span<const float> s_ht{s_sp4->spindle.held_torques};
        for (std::size_t i = 0; i < h_ht.size(); ++i) {
            require_bits_f32(h_ht[i], s_ht[i], "isolation held_torques");
        }
        require_bits_f32(
            h_sp3[wu].last_efc.delivered_crank_torque_nm,
            s_sp3->last_efc.delivered_crank_torque_nm,
            "isolation delivered_crank_torque_nm");
    }

#if defined(__APPLE__)
    const std::string gpu_src_dir = (repo_root() / "native" / "src" / "gpu").string();
    require(bike_metal_init(gpu_src_dir.c_str()) == 0, "bike_metal_init failed");

    std::vector<bike::gpu::GpuSp3WorldState> m_sp3(kNwU);
    std::vector<bike::gpu::GpuSp4WorldState> m_sp4(kNwU);
    std::vector<bike::gpu::GpuWorldState> m_ws(kNwU);
    std::vector<float> m_qpos = qpos_b;
    std::vector<float> m_qfrc(kNwU * to_index(nv), 0.0f);
    std::vector<float> m_ctrl(kNwU * to_index(nu), 0.0f);
    std::vector<float> m_tr = tr_b;
    std::vector<float> m_fl = fl_b;
    std::vector<float> m_ws_qacc = ws0_b;

    require(
        bike_metal_launch_sp4_init(
            kNworld, &idx, cfgs.data(), 1,
            m_sp3.data(), m_sp4.data(), m_ws.data(), m_qpos.data(),
            0.0f, ntendon, m_tr.data(), tr_stride) == 0,
        "metal sp4 init failed");
    require(
        bike_metal_launch_sp4_pre_solve_and_pass0(
            kNworld, kDtS, &idx, &tt->tyre, &tt->terrain_desc,
            tt->terrain_f32.data(), tt->terrain_i32.data(),
            cfgs.data(), 1, sp_params.get(), m_sp3.data(), m_sp4.data(), m_ws.data(),
            m_qpos.data(), qvel_b.data(), bias_b.data(), xpos_b.data(), xmat_b.data(),
            xa_b.data(), xx_b.data(), site_b.data(), sc_b.data(), nullptr,
            0, nM, m_ra.data(), m_rn.data(), m_ci.data(), M_b.data(),
            m_qfrc.data(), m_ctrl.data(), ntendon, m_tr.data(), tr_stride, m_fl.data(), nv,
            m_ws_qacc.data()) == 0,
        "metal sp4 pass0 failed");
    const std::vector<float> m_ctrl_p0 = m_ctrl;
    std::ranges::fill(m_ws_qacc, -999.0f);
    require(
        bike_metal_launch_sp4_pass1(
            kNworld, nq, nv, nu, sp_params.get(), m_sp4.data(),
            m_qpos.data(), qvel_b.data(), bias_b.data(), qacc_b.data(),
            0, nM, m_ra.data(), m_rn.data(), m_ci.data(), M_b.data(), m_ctrl.data(),
            m_ws_qacc.data()) == 0,
        "metal sp4 pass1 failed");
    require(m_ws_qacc == ws0_b, "metal sp4 pass1 warmstart restore mismatch");
    const std::vector<float> m_ctrl_p1 = m_ctrl;
    std::ranges::fill(m_ws_qacc, -777.0f);
    require(
        bike_metal_launch_sp4_pass2(
            kNworld, nq, nv, nu, sp_params.get(), m_sp4.data(),
            m_qpos.data(), qvel_b.data(), bias_b.data(), qacc_b.data(),
            0, nM, m_ra.data(), m_rn.data(), m_ci.data(), M_b.data(), m_ctrl.data(),
            m_ws_qacc.data()) == 0,
        "metal sp4 pass2 failed");
    require(m_ws_qacc == ws0_b, "metal sp4 pass2 warmstart restore mismatch");
    const std::vector<float> m_ctrl_p2 = m_ctrl;
    std::ranges::fill(m_ws_qacc, -555.0f);
    require(
        bike_metal_launch_sp4_pass3(
            kNworld, nq, nv, nu, sp_params.get(), m_sp4.data(),
            m_qpos.data(), qvel_b.data(), bias_b.data(), qacc_b.data(),
            0, nM, m_ra.data(), m_rn.data(), m_ci.data(), M_b.data(), m_ctrl.data(),
            m_ws_qacc.data()) == 0,
        "metal sp4 pass3 failed");
    require(m_ws_qacc == ws0_b, "metal sp4 pass3 warmstart restore mismatch");
    require(
        bike_metal_launch_sp4_post_solve(
            kNworld, kDtS, &idx, cfgs.data(), 1, sp_params.get(),
            m_sp3.data(), m_sp4.data(), m_qpos.data(), qvel_b.data(),
            xpos_b.data(), xmat_b.data(), xa_b.data(),
            nsd, sd_b.data(), nefc_b.data(), njmax, nv,
            et_b.data(), eid_b.data(), eJ_b.data(), ef_b.data(),
            nullptr, 0, 1, nullptr, nullptr, nullptr, nullptr) == 0,
        "metal sp4 post_solve failed");

    for (std::size_t i = 0; i < h_qfrc.size(); ++i) {
        require_bits_f32(m_qfrc[i], h_qfrc[i], "metal vs host qfrc");
    }
    for (std::size_t i = 0; i < h_ctrl.size(); ++i) {
        require_bits_f32(m_ctrl_p0[i], h_ctrl_p0[i], "metal vs host ctrl pass0");
        require_bits_f32(m_ctrl_p1[i], h_ctrl_p1[i], "metal vs host ctrl pass1");
        require_bits_f32(m_ctrl_p2[i], h_ctrl_p2[i], "metal vs host ctrl pass2");
        require_bits_f32(m_ctrl[i], h_ctrl[i], "metal vs host ctrl pass3");
    }
    for (std::size_t i = 0; i < h_tr.size(); ++i) {
        require_bits_f32(m_tr[i], h_tr[i], "metal vs host tendon_range");
    }
    for (std::size_t i = 0; i < h_fl.size(); ++i) {
        require_bits_f32(m_fl[i], h_fl[i], "metal vs host dof_frictionloss");
    }
    for (int w = 0; w < kNworld; ++w) {
        const std::size_t wu = to_index(w);
        const std::span<const float> m_ht{m_sp4[wu].spindle.held_torques};
        const std::span<const float> h_ht{h_sp4[wu].spindle.held_torques};
        for (std::size_t k = 0; k < m_ht.size(); ++k) {
            require_bits_f32(m_ht[k], h_ht[k], "metal vs host held_torques");
        }
        const std::span<const float> m_fp{m_sp4[wu].scratch.P.f_passive};
        const std::span<const float> h_fp{h_sp4[wu].scratch.P.f_passive};
        for (std::size_t s = 0; s < m_fp.size(); ++s) {
            require_bits_f32(m_fp[s], h_fp[s], "metal vs host P.f_passive");
        }
        require_bits_f32(
            m_sp3[wu].last_efc.delivered_crank_torque_nm,
            h_sp3[wu].last_efc.delivered_crank_torque_nm,
            "metal vs host delivered_crank_torque_nm");
        require_bits_f32(
            m_sp4[wu].min_press_front_n,
            h_sp4[wu].min_press_front_n,
            "metal vs host min_press_front_n");
        require_bits_f32(
            m_sp4[wu].min_saddle_force_n,
            h_sp4[wu].min_saddle_force_n,
            "metal vs host min_saddle_force_n");
    }
    std::cout << "[test_host_vs_metal_and_multiworld_isolation] 0-bit parity + isolation passed (nworld="
              << kNworld << ", device=" << bike_metal_device_name() << ")\n";
#endif
}

void test_short_closed_loop_rollout(const TempSp4Models &models) {
    const fs::path root = repo_root();
    const fs::path physics_path = root / "examples" / "research" / "viewer_physics_welded.toml";
    const fs::path track_path = root / "tests" / "fixtures" / "tracks" / "flat.toml";
    const auto spec = std::make_unique<bike::golden::StepClosureCaptureSpec>(
        bike::golden::StepClosureCaptureSpec{
            .physics = bike::configuration::load_physics(physics_path),
            .track = bike::terrain::load_track(track_path),
            .duration_s = std::nullopt,
            .max_steps = std::size_t{1},
            .seed_decimal = "0",
            .physics_config_label = "viewer_physics_welded.toml",
            .track_label = "flat.toml",
            .asset_root = physics_path.parent_path(),
            .strict = false,
        });
    const auto bundle = bike::golden::capture_step_closure_bundle(*spec);
    require(!bundle->rows.empty(), "capture_step_closure_bundle returned no rows");

    const fs::path rollout_dir = models.dir / "rollout";
    fs::create_directories(rollout_dir);
    const fs::path rollout_xml = rollout_dir / "model_12gears.xml";
    write_text(rollout_xml, bundle->model_12gears_xml);
    if (!bundle->hfield_data.empty()) {
        std::ofstream ofs(rollout_dir / "hfield_data.bin", std::ios::binary);
        require(ofs.is_open(), "open rollout hfield_data.bin");
        for (const double v : bundle->hfield_data) {
            const auto bytes = std::bit_cast<std::array<char, sizeof(double)>>(v);
            ofs.write(bytes.data(), sizeof(double));
        }
        require(ofs.good(), "write rollout hfield_data.bin");
    }

    ModelPtr m = load_xml(rollout_xml);
    require(bike::gpu::assets::load_hfield_bin(m.get(), rollout_xml.string().c_str()) ==
                bike::gpu::assets::AssetStatus::kOk,
            "load rollout hfield_data.bin failed");
    DataPtr d(mj_makeData(m.get()), &mj_deleteData);
    require(d != nullptr, "mj_makeData failed");
    const auto idx = bike::gpu::assets::resolve_gpu_model_indices(m.get());
    {
        const auto &row0 = bundle->rows.front();
        require(static_cast<int>(row0.qpos.size()) == m->nq &&
                    static_cast<int>(row0.qvel.size()) == m->nv,
                "bundle state size mismatch");
        std::ranges::copy(row0.qpos, ma::mutable_buffer(d->qpos, m->nq, "qpos").begin());
        std::ranges::copy(row0.qvel, ma::mutable_buffer(d->qvel, m->nv, "qvel").begin());
        if (static_cast<int>(row0.qacc_warmstart.size()) == m->nv) {
            std::ranges::copy(row0.qacc_warmstart,
                              ma::mutable_buffer(d->qacc_warmstart, m->nv, "qacc_warmstart").begin());
        }
    }

    const auto sp_params = std::make_unique<bike::physics_core::SpindleM2K2Params<float>>();
    require(
        bike::gpu::build_runner_spindle_params(
            rollout_xml.string().c_str(),
            models.strength_json.string().c_str(),
            models.envelope_json.string().c_str(),
            1, sp_params.get()) == 0,
        "build_runner_spindle_params failed");
    const auto tt = build_tyre_terrain(rollout_xml, idx);
    const auto cfg = build_sp4_sp3_config(m.get());

    const auto sp3_st = std::make_unique<bike::gpu::GpuSp3WorldState>();
    auto sp4_st = std::make_unique<bike::gpu::GpuSp4WorldState>();
    const auto ws = std::make_unique<bike::gpu::GpuWorldState>();

    const int nq = idx.nq;
    const int nv = idx.nv;
    const int nu = idx.nu;
    const int nbody = idx.nbody;
    const int njnt = to_int(m->njnt);
    const int nsite = to_int(m->nsite);
    const int ntendon = to_int(m->ntendon);
    const int nM = to_int(m->nM);
    const int nsd = to_int(m->nsensordata);

    std::vector<float> qpos_f, qvel_f, bias_f, qacc_f, xpos_f, xmat_f, xa_f, xx_f, site_f, sc_f, M_f;
    std::vector<float> qfrc_f(to_index(nv), 0.0f);
    std::vector<float> ctrl_f(to_index(nu), 0.0f);
    std::vector<float> tr_f, fl_f, sd_f, eJ_f, ef_f;
    std::vector<int> et_i, eid_i, m_ra, m_rn, m_ci;
    copy_cast(d->qpos, to_index(nq), qpos_f);
    copy_cast(m->tendon_range, 2U * to_index(ntendon), tr_f);
    copy_cast(m->dof_frictionloss, to_index(nv), fl_f);
    copy_cast(m->M_rowadr, to_index(nv), m_ra);
    copy_cast(m->M_rownnz, to_index(nv), m_rn);
    copy_cast(m->M_colind, to_index(nM), m_ci);

    const float x0 = qpos_f[to_index(idx.root_x_qpos)];
    require(
        bike_gpu_launch_sp4_init(
            nullptr, 1, &idx, cfg.get(), 0, sp3_st.get(), sp4_st.get(), ws.get(),
            qpos_f.data(), 0.0f, ntendon, tr_f.data(), 2 * ntendon) == 0,
        "rollout init failed");
    {
        const auto m_tr = ma::mutable_buffer(m->tendon_range, 2 * m->ntendon, "tendon_range");
        for (int i = 0; i < 2 * ntendon; ++i) {
            m_tr[to_index(i)] = static_cast<mjtNum>(tr_f[to_index(i)]);
        }
    }

    float max_leg_ctrl = 0.0f;
    float max_motor_ctrl = 0.0f;
    std::vector<double> ws_saved(to_index(nv), 0.0);

    constexpr int kSteps = 800;
    for (int step = 0; step < kSteps; ++step) {
        mj_fwdPosition(m.get(), d.get());
        mj_fwdVelocity(m.get(), d.get());

        copy_cast(d->qpos, to_index(nq), qpos_f);
        copy_cast(d->qvel, to_index(nv), qvel_f);
        copy_cast(d->qfrc_bias, to_index(nv), bias_f);
        copy_cast(d->xpos, to_index(nbody) * 3U, xpos_f);
        copy_cast(d->xmat, to_index(nbody) * 9U, xmat_f);
        copy_cast(d->xanchor, to_index(njnt) * 3U, xa_f);
        copy_cast(d->xaxis, to_index(njnt) * 3U, xx_f);
        copy_cast(d->site_xpos, to_index(nsite) * 3U, site_f);
        copy_cast(d->subtree_com, to_index(nbody) * 3U, sc_f);
        copy_cast(d->M, to_index(nM), M_f);
        copy_cast(m->tendon_range, 2U * to_index(ntendon), tr_f);
        copy_cast(m->dof_frictionloss, to_index(nv), fl_f);

        const int rc_r = bike_gpu_launch_sp4_pre_solve_and_pass0(
            nullptr, 1, kDtS, &idx, &tt->tyre, &tt->terrain_desc,
            tt->terrain_f32.data(), tt->terrain_i32.data(),
            cfg.get(), 0, sp_params.get(), sp3_st.get(), sp4_st.get(), ws.get(),
            qpos_f.data(), qvel_f.data(), bias_f.data(), xpos_f.data(), xmat_f.data(),
            xa_f.data(), xx_f.data(), site_f.data(), sc_f.data(), nullptr,
            0, nM, m_ra.data(), m_rn.data(), m_ci.data(), M_f.data(),
            qfrc_f.data(), ctrl_f.data(), ntendon, tr_f.data(), 2 * ntendon, fl_f.data(), nv);
        require(rc_r == 0, std::format("rollout pass0 failed at step={} rc={}", step, rc_r));

        {
            const auto d_qf = ma::mutable_buffer(d->qfrc_applied, m->nv, "qfrc_applied");
            const auto m_tr = ma::mutable_buffer(m->tendon_range, 2 * m->ntendon, "tendon_range");
            const auto m_fl = ma::mutable_buffer(m->dof_frictionloss, m->nv, "dof_frictionloss");
            for (int i = 0; i < nv; ++i) {
                d_qf[to_index(i)] = static_cast<mjtNum>(qfrc_f[to_index(i)]);
                m_fl[to_index(i)] = static_cast<mjtNum>(fl_f[to_index(i)]);
            }
            for (int i = 0; i < 2 * ntendon; ++i) {
                m_tr[to_index(i)] = static_cast<mjtNum>(tr_f[to_index(i)]);
            }
        }
        mj_makeConstraint(m.get(), d.get());
        mj_island(m.get(), d.get());
        mj_projectConstraint(m.get(), d.get());
        mj_referenceConstraint(m.get(), d.get());

        {
            const auto d_ws = ma::readonly_buffer(d->qacc_warmstart, m->nv, "qacc_warmstart");
            std::ranges::copy(d_ws, ws_saved.begin());
        }
        for (int pass = 0; pass < 4; ++pass) {
            if (pass > 0) {
                std::ranges::copy(ws_saved, ma::mutable_buffer(d->qacc_warmstart, m->nv, "qacc_warmstart").begin());
            }
            const auto d_ctrl = ma::mutable_buffer(d->ctrl, m->nu, "ctrl");
            for (int i = 0; i < nu; ++i) {
                d_ctrl[to_index(i)] = static_cast<mjtNum>(ctrl_f[to_index(i)]);
            }
            mj_fwdActuation(m.get(), d.get());
            mj_fwdAcceleration(m.get(), d.get());
            mj_fwdConstraint(m.get(), d.get());
            copy_cast(d->qacc, to_index(nv), qacc_f);

            if (pass == 0) {
                require(
                    bike_gpu_launch_sp4_pass1(
                        nullptr, 1, nq, nv, nu, sp_params.get(), sp4_st.get(),
                        qpos_f.data(), qvel_f.data(), bias_f.data(), qacc_f.data(),
                        0, nM, m_ra.data(), m_rn.data(), m_ci.data(), M_f.data(), ctrl_f.data()) == 0,
                    "rollout pass1 failed");
            } else if (pass == 1) {
                require(
                    bike_gpu_launch_sp4_pass2(
                        nullptr, 1, nq, nv, nu, sp_params.get(), sp4_st.get(),
                        qpos_f.data(), qvel_f.data(), bias_f.data(), qacc_f.data(),
                        0, nM, m_ra.data(), m_rn.data(), m_ci.data(), M_f.data(), ctrl_f.data()) == 0,
                    "rollout pass2 failed");
            } else if (pass == 2) {
                require(
                    bike_gpu_launch_sp4_pass3(
                        nullptr, 1, nq, nv, nu, sp_params.get(), sp4_st.get(),
                        qpos_f.data(), qvel_f.data(), bias_f.data(), qacc_f.data(),
                        0, nM, m_ra.data(), m_rn.data(), m_ci.data(), M_f.data(), ctrl_f.data()) == 0,
                    "rollout pass3 failed");
            }
        }

        for (int k = 8; k <= 11; ++k) {
            max_leg_ctrl = std::max(max_leg_ctrl, std::abs(ctrl_f[to_index(k)]));
        }
        max_motor_ctrl = std::max(max_motor_ctrl, ctrl_f[to_index(cfg->motor_actuator_idx)]);

        mj_sensorAcc(m.get(), d.get());
        mj_step2(m.get(), d.get());

        copy_cast(d->qpos, to_index(nq), qpos_f);
        copy_cast(d->qvel, to_index(nv), qvel_f);
        copy_cast(d->sensordata, to_index(nsd), sd_f);
        copy_cast(d->efc_type, to_index(std::max(1, d->nefc)), et_i);
        copy_cast(d->efc_id, to_index(std::max(1, d->nefc)), eid_i);
        copy_cast(d->efc_J, to_index(std::max(1, d->nefc)) * to_index(nv), eJ_f);
        copy_cast(d->efc_force, to_index(std::max(1, d->nefc)), ef_f);

        require(
            bike_gpu_launch_sp4_post_solve(
                nullptr, 1, kDtS, &idx, cfg.get(), 0, sp_params.get(),
                sp3_st.get(), sp4_st.get(), qpos_f.data(), qvel_f.data(),
                xpos_f.data(), xmat_f.data(), xa_f.data(),
                nsd, sd_f.data(), &d->nefc, std::max(1, d->nefc), nv,
                et_i.data(), eid_i.data(), eJ_f.data(), ef_f.data(),
                nullptr, 0, 1, nullptr, nullptr, nullptr, nullptr) == 0,
            "rollout post_solve failed");

        const auto d_qp = ma::mutable_buffer(d->qpos, m->nq, "qpos");
        for (int i = 0; i < nq; ++i) {
            require(std::isfinite(qpos_f[to_index(i)]), "nonfinite qpos during rollout");
            d_qp[to_index(i)] = static_cast<mjtNum>(qpos_f[to_index(i)]);
        }
    }

    const float mean_delivered =
        sp4_st->sum_delivered_crank_torque_nm / static_cast<float>(kSteps);
    const float dist = sp4_st->end_root_x_m - x0;
    std::cout << "[test_short_closed_loop_rollout] steps=" << kSteps
              << " max_leg_ctrl=" << max_leg_ctrl
              << " mean_delivered_crank_nm=" << mean_delivered
              << " max_motor_ctrl=" << max_motor_ctrl
              << " dist=" << dist << '\n';
    require(max_leg_ctrl > 1.0f, "legs must be actively actuated");
    require(mean_delivered > 5.0f, "delivered_crank_torque_nm must be positive");
    require(max_motor_ctrl > 5.0f, "Bosch CX assist must engage from delivered crank torque");
    require(dist > 0.05f, "bike must travel forward under closed-loop leg drive");
}

void test_failure_and_stride_guards(const TempSp4Models &models) {
    // B3: bike_gpu_sp3_state_has_failure must detect latched sp3.monitors.crash.cause (pitch_over).
    auto sp3_clean = std::make_unique<bike::gpu::GpuSp3WorldState>();
    require(bike_gpu_sp3_state_has_failure(sp3_clean.get()) == 0,
            "clean GpuSp3WorldState should not report failure");
    sp3_clean->sp3.monitors.crash.cause = bike::physics_core::CrashCause::pitch_over;
    require(bike_gpu_sp3_state_has_failure(sp3_clean.get()) == 1,
            "latched CrashCause::pitch_over must report failure");

    // M1: tendon_range_world_stride == 0 && nworld > 1 must be rejected cleanly.
    const ModelPtr m13 = load_xml(models.xml13);
    const auto idx = bike::gpu::assets::resolve_gpu_model_indices(m13.get());
    const int nq = idx.nq;
    const int ntendon = to_int(m13->ntendon);
    constexpr int kNworldNeg = 2;
    const auto cfg = build_sp4_sp3_config(m13.get());
    std::vector<bike::gpu::GpuSp3WorldState> h_sp3(to_index(kNworldNeg));
    std::vector<bike::gpu::GpuSp4WorldState> h_sp4(to_index(kNworldNeg));
    std::vector<bike::gpu::GpuWorldState> h_ws(to_index(kNworldNeg));
    std::vector<float> qpos_n(to_index(kNworldNeg) * to_index(nq), 0.0f);
    std::vector<float> tr_n(2U * to_index(ntendon), 0.0f);
    require(
        bike_gpu_launch_sp4_init(
            nullptr, kNworldNeg, &idx, cfg.get(), 0,
            h_sp3.data(), h_sp4.data(), h_ws.data(),
            qpos_n.data(), 0.0f, ntendon, tr_n.data(), 0) < 0,
        "host sp4_init must reject tendon_range_world_stride == 0 with nworld > 1");
#if defined(__APPLE__)
    require(
        bike_metal_launch_sp4_init(
            kNworldNeg, &idx, cfg.get(), 0,
            h_sp3.data(), h_sp4.data(), h_ws.data(),
            qpos_n.data(), 0.0f, ntendon, tr_n.data(), 0) < 0,
        "metal sp4_init must reject tendon_range_world_stride == 0 with nworld > 1");
#endif
    std::cout << "[test_failure_and_stride_guards] pitch_over + zero-stride checks passed\n";
}

[[gnu::noinline]] void test_sp4_m0_host_vs_metal_parity(const TempSp4Models &models) {
    require(bike_gpu_sizeof_spindle_m0_scratch() ==
                static_cast<int>(sizeof(bike::physics_core::SpindleM0Scratch<float>)),
            "sizeof(SpindleM0Scratch<float>) mismatch");
    require(bike_gpu_sizeof_pneumatic_scratch() ==
                static_cast<int>(sizeof(bike::physics_core::GpuPneumaticStepScratch<float>)),
            "sizeof(GpuPneumaticStepScratch<float>) mismatch");

    const ModelPtr m13 = load_xml(models.xml13);
    const auto idx = bike::gpu::assets::resolve_gpu_model_indices(m13.get());
    const auto sp_params = std::make_unique<bike::physics_core::SpindleM2K2Params<float>>();
    require(
        bike::gpu::build_runner_spindle_params(
            models.xml13.string().c_str(),
            models.strength_json.string().c_str(),
            models.envelope_json.string().c_str(),
            1, sp_params.get()) == 0,
        "m0 parity build_runner_spindle_params failed");
    const auto tt = build_tyre_terrain(models.xml13, idx);
    const auto base_cfg = build_sp4_sp3_config(m13.get());

    DataPtr d0(mj_makeData(m13.get()), &mj_deleteData);
    require(d0 != nullptr, "mj_makeData failed");
    {
        std::vector<double> qp;
        std::vector<double> qv;
        if (bike::gpu::assets::load_captured_initial_state(
                models.xml13.string().c_str(), idx.nq, idx.nv, qp, qv) ==
            bike::gpu::assets::AssetStatus::kOk) {
            std::ranges::copy(qp, ma::mutable_buffer(d0->qpos, m13->nq, "qpos").begin());
            std::ranges::copy(qv, ma::mutable_buffer(d0->qvel, m13->nv, "qvel").begin());
        }
        const auto d_qp = ma::mutable_buffer(d0->qpos, m13->nq, "qpos");
        const auto d_qv = ma::mutable_buffer(d0->qvel, m13->nv, "qvel");
        d_qp[to_index(idx.root_x_qpos)] = 2.0;
        d_qv[to_index(idx.root_x_dof)] = 2.0;
        d_qv[to_index(idx.crank_spin_dof)] = 6.0;
    }
    mj_fwdPosition(m13.get(), d0.get());
    mj_fwdVelocity(m13.get(), d0.get());
    mj_fwdActuation(m13.get(), d0.get());
    mj_fwdAcceleration(m13.get(), d0.get());
    mj_fwdConstraint(m13.get(), d0.get());

    constexpr int kNworld = 8;
    const std::size_t kNwU = to_index(kNworld);
    const int nq = idx.nq;
    const int nv = idx.nv;
    const int nu = idx.nu;
    const int nbody = idx.nbody;
    const int njnt = to_int(m13->njnt);
    const int nsite = to_int(m13->nsite);
    const int ntendon = to_int(m13->ntendon);
    const int tr_stride = 2 * ntendon;
    const int nsd = to_int(m13->nsensordata);
    const int nM = to_int(m13->nM);
    const int njmax = std::max(1, d0->nefc);

    std::vector<float> p_qpos, p_qvel, p_bias, p_qacc, p_xpos, p_xmat, p_xa, p_xx, p_site, p_sc, p_sm, p_M;
    std::vector<float> p_tr, p_fl, p_sd, p_eJ, p_ef, p_bmass, p_xipos;
    std::vector<int> p_et, p_eid, m_ra, m_rn, m_ci;
    copy_cast(d0->qpos, to_index(nq), p_qpos);
    copy_cast(d0->qvel, to_index(nv), p_qvel);
    copy_cast(d0->qfrc_bias, to_index(nv), p_bias);
    copy_cast(d0->qacc, to_index(nv), p_qacc);
    copy_cast(d0->xpos, to_index(nbody) * 3U, p_xpos);
    copy_cast(d0->xmat, to_index(nbody) * 9U, p_xmat);
    copy_cast(d0->xanchor, to_index(njnt) * 3U, p_xa);
    copy_cast(d0->xaxis, to_index(njnt) * 3U, p_xx);
    copy_cast(d0->site_xpos, to_index(nsite) * 3U, p_site);
    copy_cast(d0->subtree_com, to_index(nbody) * 3U, p_sc);
    copy_cast(m13->body_subtreemass, to_index(nbody), p_sm);
    copy_cast(m13->body_mass, to_index(nbody), p_bmass);
    copy_cast(d0->xipos, to_index(nbody) * 3U, p_xipos);
    copy_cast(d0->M, to_index(nM), p_M);
    copy_cast(m13->tendon_range, to_index(tr_stride), p_tr);
    copy_cast(m13->dof_frictionloss, to_index(nv), p_fl);
    copy_cast(d0->sensordata, to_index(nsd), p_sd);
    copy_cast(d0->efc_J, to_index(njmax) * to_index(nv), p_eJ);
    copy_cast(d0->efc_force, to_index(njmax), p_ef);
    copy_cast(d0->efc_type, to_index(njmax), p_et);
    copy_cast(d0->efc_id, to_index(njmax), p_eid);
    copy_cast(m13->M_rowadr, to_index(nv), m_ra);
    copy_cast(m13->M_rownnz, to_index(nv), m_rn);
    copy_cast(m13->M_colind, to_index(nM), m_ci);
    std::array<float, 3> grav_f{
        static_cast<float>(std::span{m13->opt.gravity}[0]),
        static_cast<float>(std::span{m13->opt.gravity}[1]),
        static_cast<float>(std::span{m13->opt.gravity}[2]),
    };

    const auto tile = [&](const auto &proto) {
        using ElemT = typename std::decay_t<decltype(proto)>::value_type;
        std::vector<ElemT> out(kNwU * proto.size());
        for (int w = 0; w < kNworld; ++w) {
            std::ranges::copy(proto, out.begin() + static_cast<std::ptrdiff_t>(to_index(w) * proto.size()));
        }
        return out;
    };

    std::vector<bike::gpu::GpuSp3Config> cfgs(kNwU, *base_cfg);
    auto qpos_b = tile(p_qpos);
    for (int w = 0; w < kNworld; ++w) {
        cfgs[to_index(w)].tire_stiff_scale = 0.90f + 0.02f * static_cast<float>(w);
        qpos_b[to_index(w) * to_index(nq) + to_index(idx.crank_spin_qpos)] += 0.05f * static_cast<float>(w);
    }
    const auto qvel_b = tile(p_qvel);
    const auto bias_b = tile(p_bias);
    const auto xpos_b = tile(p_xpos);
    const auto xmat_b = tile(p_xmat);
    const auto xa_b = tile(p_xa);
    const auto xx_b = tile(p_xx);
    const auto site_b = tile(p_site);
    const auto sc_b = tile(p_sc);
    const auto xipos_b = tile(p_xipos);
    const auto M_b = tile(p_M);
    const auto tr_b = tile(p_tr);
    const auto fl_b = tile(p_fl);
    const auto sd_b = tile(p_sd);
    const auto eJ_b = tile(p_eJ);
    const auto ef_b = tile(p_ef);
    const auto et_b = tile(p_et);
    const auto eid_b = tile(p_eid);
    const std::vector<int> nefc_b(kNwU, d0->nefc);

    std::vector<bike::gpu::GpuSp3WorldState> h_sp3(kNwU);
    std::vector<bike::gpu::GpuSp4WorldState> h_sp4(kNwU);
    std::vector<bike::physics_core::SpindleM0Scratch<float>> h_m0(kNwU);
    std::vector<bike::gpu::GpuWorldState> h_ws(kNwU);
    std::vector<float> h_qpos = qpos_b;
    std::vector<float> h_qfrc(kNwU * to_index(nv), 0.0f);
    std::vector<float> h_ctrl(kNwU * to_index(nu), 0.0f);
    std::vector<float> h_tr = tr_b;
    std::vector<float> h_fl = fl_b;
    std::vector<float> h_ws_qacc(kNwU * to_index(nv), 0.0f);

    require(
        bike_gpu_launch_sp4_m0_init(
            nullptr, kNworld, &idx, cfgs.data(), 1,
            h_sp3.data(), h_sp4.data(), h_m0.data(), h_ws.data(), h_qpos.data(),
            0.0f, ntendon, h_tr.data(), tr_stride) == 0,
        "host m0 init failed");

#if defined(__APPLE__)
    const std::string gpu_src_dir = (repo_root() / "native" / "src" / "gpu").string();
    require(bike_metal_init(gpu_src_dir.c_str()) == 0, "bike_metal_init failed");

    std::vector<bike::gpu::GpuSp3WorldState> m_sp3(kNwU);
    std::vector<bike::gpu::GpuSp4WorldState> m_sp4(kNwU);
    std::vector<bike::physics_core::SpindleM0Scratch<float>> m_m0(kNwU);
    std::vector<bike::gpu::GpuWorldState> m_ws(kNwU);
    std::vector<float> m_qpos = qpos_b;
    std::vector<float> m_qfrc(kNwU * to_index(nv), 0.0f);
    std::vector<float> m_ctrl(kNwU * to_index(nu), 0.0f);
    std::vector<float> m_tr = tr_b;
    std::vector<float> m_fl = fl_b;
    std::vector<float> m_ws_qacc(kNwU * to_index(nv), 0.0f);

    require(
        bike_metal_launch_sp4_m0_init(
            kNworld, &idx, cfgs.data(), 1,
            m_sp3.data(), m_sp4.data(), m_m0.data(), m_ws.data(), m_qpos.data(),
            0.0f, ntendon, m_tr.data(), tr_stride) == 0,
        "metal m0 init failed");
#endif

    int h_any_needs = 0;
    require(
        bike_gpu_launch_sp4_m0_pre_solve_and_begin(
            nullptr, kNworld, kDtS, &idx, &tt->tyre, &tt->terrain_desc,
            tt->terrain_f32.data(), tt->terrain_i32.data(),
            cfgs.data(), 1, sp_params.get(), h_sp3.data(), h_sp4.data(), h_m0.data(), h_ws.data(),
            h_qpos.data(), qvel_b.data(), bias_b.data(), xpos_b.data(), xmat_b.data(),
            xa_b.data(), xx_b.data(), site_b.data(), sc_b.data(), p_sm.data(),
            0, nM, m_ra.data(), m_rn.data(), m_ci.data(), M_b.data(),
            h_qfrc.data(), h_ctrl.data(), ntendon, h_tr.data(), tr_stride, h_fl.data(), nv,
            h_ws_qacc.data(), &h_any_needs) == 0,
        "host m0 pre_solve_and_begin failed");

#if defined(__APPLE__)
    int m_any_needs = 0;
    require(
        bike_metal_launch_sp4_m0_pre_solve_and_begin(
            kNworld, kDtS, &idx, &tt->tyre, &tt->terrain_desc,
            tt->terrain_f32.data(), tt->terrain_i32.data(),
            cfgs.data(), 1, sp_params.get(), m_sp3.data(), m_sp4.data(), m_m0.data(), m_ws.data(),
            m_qpos.data(), qvel_b.data(), bias_b.data(), xpos_b.data(), xmat_b.data(),
            xa_b.data(), xx_b.data(), site_b.data(), sc_b.data(), p_sm.data(),
            0, nM, m_ra.data(), m_rn.data(), m_ci.data(), M_b.data(),
            m_qfrc.data(), m_ctrl.data(), ntendon, m_tr.data(), tr_stride, m_fl.data(), nv,
            m_ws_qacc.data(), &m_any_needs) == 0,
        "metal m0 pre_solve_and_begin failed");
    require(m_any_needs == h_any_needs, "metal vs host m0 begin any_needs_solve mismatch");
#endif

    // Drive multi-iteration Armijo Newton passes using per-world MuJoCo acceleration response.
    std::vector<float> qacc_b(kNwU * to_index(nv), 0.0f);
    std::vector<DataPtr> world_d;
    world_d.reserve(kNwU);
    std::vector<std::vector<double>> ws_saved_w(kNwU, std::vector<double>(to_index(nv), 0.0));
    for (int w = 0; w < kNworld; ++w) {
        const std::size_t wu = to_index(w);
        DataPtr dw(mj_makeData(m13.get()), &mj_deleteData);
        const auto d_qp = ma::mutable_buffer(dw->qpos, m13->nq, "qpos");
        const auto d_qv = ma::mutable_buffer(dw->qvel, m13->nv, "qvel");
        const auto d_qf = ma::mutable_buffer(dw->qfrc_applied, m13->nv, "qfrc_applied");
        const auto mj_tr = ma::mutable_buffer(m13->tendon_range, 2 * m13->ntendon, "tendon_range");
        const auto mj_fl = ma::mutable_buffer(m13->dof_frictionloss, m13->nv, "dof_frictionloss");
        for (int i = 0; i < nq; ++i) {
            d_qp[to_index(i)] = static_cast<mjtNum>(h_qpos[wu * to_index(nq) + to_index(i)]);
        }
        for (int i = 0; i < nv; ++i) {
            d_qv[to_index(i)] = static_cast<mjtNum>(qvel_b[wu * to_index(nv) + to_index(i)]);
            d_qf[to_index(i)] = static_cast<mjtNum>(h_qfrc[wu * to_index(nv) + to_index(i)]);
            mj_fl[to_index(i)] = static_cast<mjtNum>(h_fl[wu * to_index(nv) + to_index(i)]);
        }
        for (int i = 0; i < 2 * ntendon; ++i) {
            mj_tr[to_index(i)] = static_cast<mjtNum>(h_tr[wu * to_index(tr_stride) + to_index(i)]);
        }
        mj_fwdPosition(m13.get(), dw.get());
        mj_fwdVelocity(m13.get(), dw.get());
        mj_makeConstraint(m13.get(), dw.get());
        mj_island(m13.get(), dw.get());
        mj_projectConstraint(m13.get(), dw.get());
        mj_referenceConstraint(m13.get(), dw.get());
        std::ranges::copy(
            ma::readonly_buffer(dw->qacc_warmstart, m13->nv, "qacc_warmstart"),
            ws_saved_w[wu].begin());
        world_d.push_back(std::move(dw));
    }

    int iter = 0;
    while (h_any_needs != 0 && iter < bike::physics_core::kM0MaxPasses) {
        for (int w = 0; w < kNworld; ++w) {
            const std::size_t wu = to_index(w);
            mjData *dw = world_d[wu].get();
            const auto mj_tr = ma::mutable_buffer(m13->tendon_range, 2 * m13->ntendon, "tendon_range");
            const auto mj_fl = ma::mutable_buffer(m13->dof_frictionloss, m13->nv, "dof_frictionloss");
            for (int i = 0; i < nv; ++i) {
                mj_fl[to_index(i)] = static_cast<mjtNum>(h_fl[wu * to_index(nv) + to_index(i)]);
            }
            for (int i = 0; i < 2 * ntendon; ++i) {
                mj_tr[to_index(i)] = static_cast<mjtNum>(h_tr[wu * to_index(tr_stride) + to_index(i)]);
            }
            const auto d_ctrl = ma::mutable_buffer(dw->ctrl, m13->nu, "ctrl");
            for (int i = 0; i < nu; ++i) {
                d_ctrl[to_index(i)] = static_cast<mjtNum>(h_ctrl[wu * to_index(nu) + to_index(i)]);
            }
            std::ranges::copy(
                ws_saved_w[wu],
                ma::mutable_buffer(dw->qacc_warmstart, m13->nv, "qacc_warmstart").begin());
            mj_fwdActuation(m13.get(), dw);
            mj_fwdAcceleration(m13.get(), dw);
            mj_fwdConstraint(m13.get(), dw);
            const auto d_qacc = ma::readonly_buffer(dw->qacc, m13->nv, "qacc");
            for (int i = 0; i < nv; ++i) {
                qacc_b[wu * to_index(nv) + to_index(i)] = static_cast<float>(d_qacc[to_index(i)]);
            }
        }

        require(
            bike_gpu_launch_sp4_m0_step(
                nullptr, kNworld, nq, nv, nu, &idx, cfgs.data(), 1, h_sp3.data(),
                sp_params.get(), h_sp4.data(), h_m0.data(),
                h_qpos.data(), qvel_b.data(), bias_b.data(), qacc_b.data(),
                0, nM, m_ra.data(), m_rn.data(), m_ci.data(), M_b.data(),
                h_ctrl.data(), h_ws_qacc.data(), &h_any_needs) == 0,
            "host m0 step failed");
#if defined(__APPLE__)
        require(
            bike_metal_launch_sp4_m0_step(
                kNworld, nq, nv, nu, &idx, cfgs.data(), 1, m_sp3.data(),
                sp_params.get(), m_sp4.data(), m_m0.data(),
                m_qpos.data(), qvel_b.data(), bias_b.data(), qacc_b.data(),
                0, nM, m_ra.data(), m_rn.data(), m_ci.data(), M_b.data(),
                m_ctrl.data(), m_ws_qacc.data(), &m_any_needs) == 0,
            "metal m0 step failed");
        require(m_any_needs == h_any_needs, "metal vs host m0 step any_needs_solve mismatch");
        for (std::size_t i = 0; i < h_ctrl.size(); ++i) {
            require_ulp_f32(m_ctrl[i], h_ctrl[i], 1U, "metal vs host m0 step ctrl");
        }
#endif
        ++iter;
    }
    require(iter >= 1 && h_any_needs == 0,
            std::format("SpindleM0 parity rollout must converge (iter={}, h_any_needs={})",
                        iter, h_any_needs));

    require(
        bike_gpu_launch_sp4_m0_post_solve(
            nullptr, kNworld, kDtS, &idx, cfgs.data(), 1, sp_params.get(),
            h_sp3.data(), h_sp4.data(), h_m0.data(), h_qpos.data(), qvel_b.data(),
            xpos_b.data(), xmat_b.data(), xa_b.data(),
            nsd, sd_b.data(), nefc_b.data(), njmax, nv,
            et_b.data(), eid_b.data(), eJ_b.data(), ef_b.data(),
            nullptr, 0, 1, nullptr, nullptr, nullptr, nullptr) == 0,
        "host m0 post_solve failed");

    require(
        bike_gpu_launch_sp4_energy_post_step(
            nullptr, kNworld, kDtS, &idx, &tt->tyre, &tt->terrain_desc,
            tt->terrain_f32.data(), cfgs.data(), 1, sp_params.get(),
            h_sp3.data(), h_sp4.data(), h_ws.data(),
            0, nM, m_ra.data(), m_rn.data(), m_ci.data(), M_b.data(),
            h_qpos.data(), qvel_b.data(), h_ctrl.data(),
            p_bmass.data(), xipos_b.data(), grav_f.data(),
            nefc_b.data(), njmax, nv,
            et_b.data(), eJ_b.data(), ef_b.data()) == 0,
        "host m0 energy_post_step failed");

#if defined(__APPLE__)
    require(
        bike_metal_launch_sp4_m0_post_solve(
            kNworld, kDtS, &idx, cfgs.data(), 1, sp_params.get(),
            m_sp3.data(), m_sp4.data(), m_m0.data(), m_qpos.data(), qvel_b.data(),
            xpos_b.data(), xmat_b.data(), xa_b.data(),
            nsd, sd_b.data(), nefc_b.data(), njmax, nv,
            et_b.data(), eid_b.data(), eJ_b.data(), ef_b.data(),
            nullptr, 0, 1, nullptr, nullptr, nullptr, nullptr) == 0,
        "metal m0 post_solve failed");
    require(
        bike_metal_launch_sp4_energy_post_step(
            kNworld, kDtS, &idx, &tt->tyre, &tt->terrain_desc,
            tt->terrain_f32.data(), cfgs.data(), 1, sp_params.get(),
            m_sp3.data(), m_sp4.data(), m_ws.data(),
            0, nM, m_ra.data(), m_rn.data(), m_ci.data(), M_b.data(),
            m_qpos.data(), qvel_b.data(), m_ctrl.data(),
            p_bmass.data(), xipos_b.data(), grav_f.data(),
            nefc_b.data(), njmax, nv,
            et_b.data(), eJ_b.data(), ef_b.data()) == 0,
        "metal m0 energy_post_step failed");

    for (std::size_t i = 0; i < h_qfrc.size(); ++i) {
        require_ulp_f32(m_qfrc[i], h_qfrc[i], 1U, "metal vs host m0 qfrc");
    }
    for (int w = 0; w < kNworld; ++w) {
        const std::size_t wu = to_index(w);
        require(m_sp4[wu].current_m0_passes == h_sp4[wu].current_m0_passes,
                "metal vs host m0 pass count mismatch");
        const std::span<const float> m_ht{m_sp4[wu].spindle.held_torques};
        const std::span<const float> h_ht{h_sp4[wu].spindle.held_torques};
        for (std::size_t k = 0; k < m_ht.size(); ++k) {
            require_ulp_f32(m_ht[k], h_ht[k], 1U, "metal vs host m0 held_torques");
        }
        require_ulp_f32(m_sp4[wu].energy_residual_j, h_sp4[wu].energy_residual_j, 1U,
                        "metal vs host energy_residual_j");
        require_ulp_f32(m_sp4[wu].energy_scale_j, h_sp4[wu].energy_scale_j, 1U,
                        "metal vs host energy_scale_j");
    }
#endif
    std::cout << "[test_sp4_m0_host_vs_metal_parity] passed (iters=" << iter << ")\n";
}

[[gnu::noinline]] void test_sp4_m0_and_energy_contract_all_tyres(const TempSp4Models &models) {
    struct BackendRun {
        const char *label;
        const char *physics_toml;
        bike::physics_core::TyreBackendKind backend;
        int station_count;
    };
    const std::array<BackendRun, 3> backends{{
        {.label = "compliant_2d",
         .physics_toml = "examples/research/viewer_physics_welded.toml",
         .backend = bike::physics_core::TyreBackendKind::compliant_2d,
         .station_count = 0},
        {.label = "distributed_2d_reference",
         .physics_toml = "examples/research/viewer_physics_distributed.toml",
         .backend = bike::physics_core::TyreBackendKind::distributed_2d_reference,
         .station_count = 64},
        {.label = "pneumatic",
         .physics_toml = "examples/research/viewer_physics_pneumatic.toml",
         .backend = bike::physics_core::TyreBackendKind::pneumatic,
         .station_count = 64},
    }};

    const fs::path root = repo_root();
    const fs::path track_path = root / "tests" / "fixtures" / "tracks" / "flat.toml";
    for (const auto &br : backends) {
        const fs::path physics_path = root / br.physics_toml;
        const auto spec = std::make_unique<bike::golden::StepClosureCaptureSpec>(
            bike::golden::StepClosureCaptureSpec{
                .physics = bike::configuration::load_physics(physics_path),
                .track = bike::terrain::load_track(track_path),
                .duration_s = std::nullopt,
                .max_steps = std::size_t{1},
                .seed_decimal = "0",
                .physics_config_label = physics_path.filename().string(),
                .track_label = "flat.toml",
                .asset_root = physics_path.parent_path(),
                .strict = false,
            });
        const auto bundle = bike::golden::capture_step_closure_bundle(*spec);
        require(!bundle->rows.empty(), "capture_step_closure_bundle returned no rows");

        const fs::path rollout_dir = models.dir / std::format("m0_{}", br.label);
        fs::create_directories(rollout_dir);
        const fs::path rollout_xml = rollout_dir / "model_12gears.xml";
        write_text(rollout_xml, bundle->model_12gears_xml);
        if (!bundle->hfield_data.empty()) {
            std::ofstream ofs(rollout_dir / "hfield_data.bin", std::ios::binary);
            require(ofs.is_open(), "open rollout hfield_data.bin");
            for (const double v : bundle->hfield_data) {
                const auto bytes = std::bit_cast<std::array<char, sizeof(double)>>(v);
                ofs.write(bytes.data(), sizeof(double));
            }
            require(ofs.good(), "write rollout hfield_data.bin");
        }

        ModelPtr m = load_xml(rollout_xml);
        require(bike::gpu::assets::load_hfield_bin(m.get(), rollout_xml.string().c_str()) ==
                    bike::gpu::assets::AssetStatus::kOk,
                "load m0 rollout hfield_data.bin failed");
        DataPtr d(mj_makeData(m.get()), &mj_deleteData);
        require(d != nullptr, "mj_makeData failed");
        const auto idx = bike::gpu::assets::resolve_gpu_model_indices(m.get());
        {
            const auto &row0 = bundle->rows.front();
            std::ranges::copy(row0.qpos, ma::mutable_buffer(d->qpos, m->nq, "qpos").begin());
            std::ranges::copy(row0.qvel, ma::mutable_buffer(d->qvel, m->nv, "qvel").begin());
            if (static_cast<int>(row0.qacc_warmstart.size()) == m->nv) {
                std::ranges::copy(row0.qacc_warmstart,
                                  ma::mutable_buffer(d->qacc_warmstart, m->nv, "qacc_warmstart").begin());
            }
        }

        const auto sp_params = std::make_unique<bike::physics_core::SpindleM2K2Params<float>>();
        require(
            bike::gpu::build_runner_spindle_params(
                rollout_xml.string().c_str(),
                models.strength_json.string().c_str(),
                models.envelope_json.string().c_str(),
                1, sp_params.get()) == 0,
            "m0 build_runner_spindle_params failed");

        auto tt = std::make_unique<FlatTerrainAndTyre>();
        {
            int f32_count = 0;
            int i32_count = 0;
            require(
                bike::gpu::build_runner_tyre_and_terrain_buffers_for_surface(
                    rollout_xml.string().c_str(),
                    static_cast<int>(br.backend),
                    br.station_count,
                    static_cast<int>(bike::physics_core::SurfaceLabel::hardpack),
                    &idx, &tt->tyre, &tt->terrain_desc,
                    nullptr, &f32_count, nullptr, &i32_count) == 0 && f32_count > 0,
                "m0 tyre/terrain size query failed");
            tt->terrain_f32.resize(to_index(f32_count));
            tt->terrain_i32.resize(to_index(std::max(1, i32_count)));
            require(
                bike::gpu::build_runner_tyre_and_terrain_buffers_for_surface(
                    rollout_xml.string().c_str(),
                    static_cast<int>(br.backend),
                    br.station_count,
                    static_cast<int>(bike::physics_core::SurfaceLabel::hardpack),
                    &idx, &tt->tyre, &tt->terrain_desc,
                    tt->terrain_f32.data(), &f32_count,
                    tt->terrain_i32.data(), &i32_count) == 0,
                "m0 tyre/terrain fill failed");
        }

        const auto cfg = build_sp4_sp3_config(m.get());
        cfg->suspension = bike::gpu::gpu_make_backend_default_suspension_params(br.backend);
        cfg->resistance.crr = static_cast<float>(spec->physics.resistance.crr);
        cfg->track_energy_ledger = 1;

        const auto sp3_st = std::make_unique<bike::gpu::GpuSp3WorldState>();
        const auto sp4_st = std::make_unique<bike::gpu::GpuSp4WorldState>();
        const auto m0_scratch = std::make_unique<bike::physics_core::SpindleM0Scratch<float>>();
        const auto ws = std::make_unique<bike::gpu::GpuWorldState>();

        const int nq = idx.nq;
        const int nv = idx.nv;
        const int nu = idx.nu;
        const int nbody = idx.nbody;
        const int njnt = to_int(m->njnt);
        const int nsite = to_int(m->nsite);
        const int ntendon = to_int(m->ntendon);
        const int nM = to_int(m->nM);
        const int nsd = to_int(m->nsensordata);

        std::vector<float> qpos_f, qvel_f, bias_f, qacc_f, xpos_f, xmat_f, xa_f, xx_f, site_f, sc_f, sm_f, bmass_f, xipos_f, M_f;
        std::vector<float> qfrc_f(to_index(nv), 0.0f);
        std::vector<float> ctrl_f(to_index(nu), 0.0f);
        std::vector<float> tr_f, fl_f, sd_f, eJ_f, ef_f;
        std::vector<float> ws_qacc_f(to_index(nv), 0.0f);
        std::vector<int> et_i, eid_i, m_ra, m_rn, m_ci;
        copy_cast(d->qpos, to_index(nq), qpos_f);
        copy_cast(m->body_subtreemass, to_index(nbody), sm_f);
        copy_cast(m->body_mass, to_index(nbody), bmass_f);
        copy_cast(m->tendon_range, 2U * to_index(ntendon), tr_f);
        copy_cast(m->dof_frictionloss, to_index(nv), fl_f);
        copy_cast(m->M_rowadr, to_index(nv), m_ra);
        copy_cast(m->M_rownnz, to_index(nv), m_rn);
        copy_cast(m->M_colind, to_index(nM), m_ci);
        std::array<float, 3> grav_f{
            static_cast<float>(std::span{m->opt.gravity}[0]),
            static_cast<float>(std::span{m->opt.gravity}[1]),
            static_cast<float>(std::span{m->opt.gravity}[2]),
        };

        const float x0 = qpos_f[to_index(idx.root_x_qpos)];
        require(
            bike_gpu_launch_sp4_m0_init(
                nullptr, 1, &idx, cfg.get(), 0,
                sp3_st.get(), sp4_st.get(), m0_scratch.get(), ws.get(),
                qpos_f.data(), 0.0f, ntendon, tr_f.data(), 2 * ntendon) == 0,
            "m0 rollout init failed");
        {
            const auto m_tr = ma::mutable_buffer(m->tendon_range, 2 * m->ntendon, "tendon_range");
            for (int i = 0; i < 2 * ntendon; ++i) {
                m_tr[to_index(i)] = static_cast<mjtNum>(tr_f[to_index(i)]);
            }
        }

        std::vector<double> ws_saved(to_index(nv), 0.0);
        constexpr int kSteps = 400;
        for (int step = 0; step < kSteps; ++step) {
            mj_fwdPosition(m.get(), d.get());
            mj_fwdVelocity(m.get(), d.get());

            copy_cast(d->qpos, to_index(nq), qpos_f);
            copy_cast(d->qvel, to_index(nv), qvel_f);
            copy_cast(d->qfrc_bias, to_index(nv), bias_f);
            copy_cast(d->xpos, to_index(nbody) * 3U, xpos_f);
            copy_cast(d->xmat, to_index(nbody) * 9U, xmat_f);
            copy_cast(d->xanchor, to_index(njnt) * 3U, xa_f);
            copy_cast(d->xaxis, to_index(njnt) * 3U, xx_f);
            copy_cast(d->site_xpos, to_index(nsite) * 3U, site_f);
            copy_cast(d->subtree_com, to_index(nbody) * 3U, sc_f);
            copy_cast(d->M, to_index(nM), M_f);
            copy_cast(m->tendon_range, 2U * to_index(ntendon), tr_f);
            copy_cast(m->dof_frictionloss, to_index(nv), fl_f);

            int any_needs = 0;
            require(
                bike_gpu_launch_sp4_m0_pre_solve_and_begin(
                    nullptr, 1, kDtS, &idx, &tt->tyre, &tt->terrain_desc,
                    tt->terrain_f32.data(), tt->terrain_i32.data(),
                    cfg.get(), 0, sp_params.get(),
                    sp3_st.get(), sp4_st.get(), m0_scratch.get(), ws.get(),
                    qpos_f.data(), qvel_f.data(), bias_f.data(), xpos_f.data(), xmat_f.data(),
                    xa_f.data(), xx_f.data(), site_f.data(), sc_f.data(), sm_f.data(),
                    0, nM, m_ra.data(), m_rn.data(), m_ci.data(), M_f.data(),
                    qfrc_f.data(), ctrl_f.data(), ntendon, tr_f.data(), 2 * ntendon,
                    fl_f.data(), nv, ws_qacc_f.data(), &any_needs) == 0,
                "m0 rollout begin failed");

            {
                const auto d_qf = ma::mutable_buffer(d->qfrc_applied, m->nv, "qfrc_applied");
                const auto m_tr = ma::mutable_buffer(m->tendon_range, 2 * m->ntendon, "tendon_range");
                const auto m_fl = ma::mutable_buffer(m->dof_frictionloss, m->nv, "dof_frictionloss");
                for (int i = 0; i < nv; ++i) {
                    d_qf[to_index(i)] = static_cast<mjtNum>(qfrc_f[to_index(i)]);
                    m_fl[to_index(i)] = static_cast<mjtNum>(fl_f[to_index(i)]);
                }
                for (int i = 0; i < 2 * ntendon; ++i) {
                    m_tr[to_index(i)] = static_cast<mjtNum>(tr_f[to_index(i)]);
                }
            }
            mj_makeConstraint(m.get(), d.get());
            mj_island(m.get(), d.get());
            mj_projectConstraint(m.get(), d.get());
            mj_referenceConstraint(m.get(), d.get());

            std::ranges::copy(ma::readonly_buffer(d->qacc_warmstart, m->nv, "qacc_warmstart"),
                              ws_saved.begin());
            int pass = 0;
            while (true) {
                if (pass > 0) {
                    std::ranges::copy(ws_saved,
                                      ma::mutable_buffer(d->qacc_warmstart, m->nv, "qacc_warmstart").begin());
                }
                const auto d_ctrl = ma::mutable_buffer(d->ctrl, m->nu, "ctrl");
                for (int i = 0; i < nu; ++i) {
                    d_ctrl[to_index(i)] = static_cast<mjtNum>(ctrl_f[to_index(i)]);
                }
                mj_fwdActuation(m.get(), d.get());
                mj_fwdAcceleration(m.get(), d.get());
                mj_fwdConstraint(m.get(), d.get());
                if (any_needs == 0 || pass >= bike::physics_core::kM0MaxPasses) {
                    break;
                }
                copy_cast(d->qacc, to_index(nv), qacc_f);
                require(
                    bike_gpu_launch_sp4_m0_step(
                        nullptr, 1, nq, nv, nu, &idx, cfg.get(), 0, sp3_st.get(),
                        sp_params.get(), sp4_st.get(), m0_scratch.get(),
                        qpos_f.data(), qvel_f.data(), bias_f.data(), qacc_f.data(),
                        0, nM, m_ra.data(), m_rn.data(), m_ci.data(), M_f.data(),
                        ctrl_f.data(), ws_qacc_f.data(), &any_needs) == 0,
                    "m0 rollout step failed");
                ++pass;
            }

            mj_sensorAcc(m.get(), d.get());
            mj_step2(m.get(), d.get());

            copy_cast(d->qpos, to_index(nq), qpos_f);
            copy_cast(d->qvel, to_index(nv), qvel_f);
            copy_cast(d->sensordata, to_index(nsd), sd_f);
            copy_cast(d->efc_type, to_index(std::max(1, d->nefc)), et_i);
            copy_cast(d->efc_id, to_index(std::max(1, d->nefc)), eid_i);
            copy_cast(d->efc_J, to_index(std::max(1, d->nefc)) * to_index(nv), eJ_f);
            copy_cast(d->efc_force, to_index(std::max(1, d->nefc)), ef_f);

            require(
                bike_gpu_launch_sp4_m0_post_solve(
                    nullptr, 1, kDtS, &idx, cfg.get(), 0, sp_params.get(),
                    sp3_st.get(), sp4_st.get(), m0_scratch.get(),
                    qpos_f.data(), qvel_f.data(), xpos_f.data(), xmat_f.data(), xa_f.data(),
                    nsd, sd_f.data(), &d->nefc, std::max(1, d->nefc), nv,
                    et_i.data(), eid_i.data(), eJ_f.data(), ef_f.data(),
                    nullptr, 0, 1, nullptr, nullptr, nullptr, nullptr) == 0,
                "m0 rollout post_solve failed");

            {
                const auto d_qp = ma::mutable_buffer(d->qpos, m->nq, "qpos");
                for (int i = 0; i < nq; ++i) {
                    require(std::isfinite(qpos_f[to_index(i)]), "nonfinite qpos in m0 rollout");
                    d_qp[to_index(i)] = static_cast<mjtNum>(qpos_f[to_index(i)]);
                }
            }
            mj_fwdPosition(m.get(), d.get());
            copy_cast(d->xipos, to_index(nbody) * 3U, xipos_f);
            copy_cast(d->M, to_index(nM), M_f);

            require(
                bike_gpu_launch_sp4_energy_post_step(
                    nullptr, 1, kDtS, &idx, &tt->tyre, &tt->terrain_desc,
                    tt->terrain_f32.data(), cfg.get(), 0, sp_params.get(),
                    sp3_st.get(), sp4_st.get(), ws.get(),
                    0, nM, m_ra.data(), m_rn.data(), m_ci.data(), M_f.data(),
                    qpos_f.data(), qvel_f.data(), ctrl_f.data(),
                    bmass_f.data(), xipos_f.data(), grav_f.data(),
                    &d->nefc, std::max(1, d->nefc), nv,
                    et_i.data(), eJ_f.data(), ef_f.data()) == 0,
                "m0 rollout energy_post_step failed");
        }

        const float dist = sp4_st->end_root_x_m - x0;
        std::cout << "[test_sp4_m0_and_energy_contract:" << br.label << "] max_m0_passes="
                  << sp4_st->max_m0_passes_used << " dist=" << dist
                  << " max_rel_energy_residual=" << sp4_st->max_rel_energy_residual
                  << " energy_contract_ok=" << static_cast<int>(sp4_st->energy_contract_ok) << '\n';
        require(sp4_st->max_m0_passes_used >= 2,
                "SpindleM0 rollout must use multi-iteration Newton passes");
        require(sp3_st->energy_ledger.initialized != 0 && sp3_st->energy_ledger.step_count >= kSteps - 1,
                "GpuEnergyLedger must advance across rollout steps");
        require(sp4_st->energy_contract_ok == 1 && sp4_st->max_rel_energy_residual <= 0.05f,
                "GpuEnergyLedger contract must hold (max_rel_energy_residual <= 0.05)");
        require(dist > 0.05f, "SpindleM0 rollout must travel forward");
    }
}

} // namespace

int main() {
    try {
        test_sizeof_parity();
        const TempSp4Models models(repo_root());
        test_failure_and_stride_guards(models);
        test_host_vs_metal_and_multiworld_isolation(models);
        test_short_closed_loop_rollout(models);
        test_sp4_m0_host_vs_metal_parity(models);
        test_sp4_m0_and_energy_contract_all_tyres(models);
        std::cout << "ALL bike_test_gpu_sp4_closed_loop tests PASSED\n";
        return 0;
    } catch (const std::exception &ex) {
        std::cerr << "FAIL: " << ex.what() << '\n';
        return 1;
    }
}
