// native/tests/gpu/test_gpu_sp3_autonomous.cpp — C++23 verification for SP3-C
// autonomous GPU closed-loop step:
// 1. 12-tendon cassette steady-gear exact equivalence across all 12 cogs + shift & coasting
// 2. Post-solve efc reaction parity (delivered_crank_torque_nm vs mj_mulJacTVec & hand_press_n)
// 3. Synchronized 2*pi epoch re-basing invariance
// 4. Host FP32 vs Apple Metal parity & 1-step nworld=1 vs nworld=64 isolation

#include "../../src/golden/step_closure_bundle.hpp"
#include "../../src/gpu/cuda_kernels_abi.hpp"
#include "../../src/gpu/model_assets.hpp"
#include "../../src/gpu/runner_tyre_setup.hpp"
#include "../../src/gpu/sp3_autonomous_step.hpp"
#include "../../src/gpu/terrain_tables.hpp"
#include "../../src/model_access.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
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

int bike_metal_sizeof_sp3_config();
int bike_metal_sizeof_sp3_world_state();
int bike_metal_init_default_sp3_config(bike::gpu::GpuSp3Config *cfg);

int bike_metal_launch_sp3_init(
    int nworld,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::gpu::GpuSp3Config *d_sp3_configs,
    int per_world_config,
    bike::gpu::GpuSp3WorldState *d_sp3_states,
    bike::gpu::GpuWorldState *d_world_states,
    const float *d_qpos,
    float init_grade,
    int ntendon,
    float *d_tendon_range,
    int tendon_range_world_stride
);

int bike_metal_launch_sp3_pre_solve(
    int nworld,
    float dt_s,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::physics_core::GpuTyreModel *d_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *d_terrain_desc,
    const float *d_terrain_f32,
    const bike::physics_core::i32 *d_terrain_i32,
    const bike::gpu::GpuSp3Config *d_sp3_configs,
    int per_world_config,
    bike::gpu::GpuSp3WorldState *d_sp3_states,
    bike::gpu::GpuWorldState *d_world_states,
    const float *d_qpos,
    const float *d_qvel,
    const float *d_xpos,
    const float *d_xmat,
    const float *d_xanchor,
    const float *d_xaxis,
    float *d_qfrc_applied,
    float *d_ctrl,
    int ntendon,
    float *d_tendon_range,
    int tendon_range_world_stride,
    float *d_dof_frictionloss,
    int dof_frictionloss_world_stride
);

int bike_metal_launch_sp3_post_solve(
    int nworld,
    float dt_s,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::gpu::GpuSp3Config *d_sp3_configs,
    int per_world_config,
    bike::gpu::GpuSp3WorldState *d_sp3_states,
    float *d_qpos,
    const float *d_qvel,
    const float *d_xmat,
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
#endif
} // extern "C"

namespace {

namespace fs = std::filesystem;
namespace model_access = ::model_access;

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
    const auto hfield_nrow = model_access::readonly_buffer(m->hfield_nrow, m->nhfield, "hfield_nrow");
    const auto hfield_ncol = model_access::readonly_buffer(m->hfield_ncol, m->nhfield, "hfield_ncol");
    const auto hfield_adr = model_access::readonly_buffer(m->hfield_adr, m->nhfield, "hfield_adr");
    const auto hfield_size = model_access::readonly_buffer(m->hfield_size, 4 * m->nhfield, "hfield_size");
    const auto geom_pos = model_access::readonly_buffer(m->geom_pos, 3 * m->ngeom, "geom_pos");
    const int nrow = hfield_nrow[to_index(hf_id)];
    const int ncol = hfield_ncol[to_index(hf_id)];
    const int adr = hfield_adr[to_index(hf_id)];
    const double sz = static_cast<double>(hfield_size[4 * to_index(hf_id) + 2]);
    const double pos_z = static_cast<double>(geom_pos[3 * to_index(terrain_geom) + 2]);
    const float flat_norm = static_cast<float>((0.0 - pos_z) / sz);
    const auto hf = model_access::mutable_buffer(m->hfield_data, m->nhfielddata, "hfield_data");
    for (int r = 0; r < nrow; ++r) {
        for (int c = 0; c < ncol; ++c) {
            hf[to_index(adr + r * ncol + c)] = flat_norm;
        }
    }
}

struct TempSp3Models {
    fs::path dir;
    fs::path xml2;
    fs::path xml13;

    explicit TempSp3Models(const fs::path &welded_src) {
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        dir = fs::temp_directory_path() / std::format("bike_sp3c_models_{}", stamp);
        fs::create_directories(dir);
        xml2 = dir / "welded.xml";
        xml13 = dir / "model_12gears.xml";

        const std::string raw_xml = read_text(welded_src);
        write_text(xml2, raw_xml);

        // Create temporary hfield_data.bin matching welded.xml flat road
        ModelPtr m_base = load_xml(welded_src);
        seed_flat_welded_road(m_base.get());
        if (m_base->nhfielddata > 0) {
            const fs::path bin_path = dir / "hfield_data.bin";
            std::ofstream ofs(bin_path, std::ios::binary);
            require(ofs.is_open(), "open temp hfield_data.bin");
            const auto hf = model_access::readonly_buffer(
                m_base->hfield_data, m_base->nhfielddata, "hfield_data");
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

    TempSp3Models(const TempSp3Models &) = delete;
    TempSp3Models &operator=(const TempSp3Models &) = delete;
    TempSp3Models(TempSp3Models &&) = delete;
    TempSp3Models &operator=(TempSp3Models &&) = delete;

    ~TempSp3Models() {
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

std::unique_ptr<FlatTerrainAndTyre> build_tyre_terrain_from_xml(
    const fs::path &xml_path,
    const bike::gpu::GpuModelIndices &idx
) {
    auto out = std::make_unique<FlatTerrainAndTyre>();
    int f32_count = 0;
    int i32_count = 0;
    const int q1 = bike::gpu::build_runner_tyre_and_terrain_buffers(
        xml_path.string().c_str(),
        static_cast<int>(bike::physics_core::TyreBackendKind::compliant_2d),
        0,
        &idx,
        &out->tyre,
        &out->terrain_desc,
        nullptr,
        &f32_count,
        nullptr,
        &i32_count);
    require(q1 == 0 && f32_count > 0 && i32_count >= 0, "terrain buffer size query failed");
    out->terrain_f32.resize(to_index(f32_count));
    out->terrain_i32.resize(to_index(std::max(1, i32_count)));
    const int q2 = bike::gpu::build_runner_tyre_and_terrain_buffers(
        xml_path.string().c_str(),
        static_cast<int>(bike::physics_core::TyreBackendKind::compliant_2d),
        0,
        &idx,
        &out->tyre,
        &out->terrain_desc,
        out->terrain_f32.data(),
        &f32_count,
        out->terrain_i32.data(),
        &i32_count);
    require(q2 == 0, "terrain buffer fill failed");
    return out;
}

bike::gpu::GpuModelIndices resolve_model_indices(const mjModel *m) {
    bike::gpu::GpuModelIndices idx = bike::gpu::assets::resolve_gpu_model_indices(m);
    require(bike::gpu::gpu_validate_model_indices(idx), "resolve_gpu_model_indices invalid");
    return idx;
}

std::unique_ptr<bike::gpu::GpuSp3Config> build_sp3_config_for_model(const mjModel *m) {
    auto cfg_ptr = std::make_unique<bike::gpu::GpuSp3Config>();
    auto &cfg = *cfg_ptr;
    bike::gpu::gpu_init_default_sp3_config(cfg);

    const auto jnt_dofadr = model_access::readonly_buffer(m->jnt_dofadr, m->njnt, "jnt_dofadr");
    const auto sensor_adr = model_access::readonly_buffer(m->sensor_adr, m->nsensor, "sensor_adr");

    const int crank_jnt = mj_name2id(m, mjOBJ_JOINT, "crank_spin");
    const int fw_jnt = mj_name2id(m, mjOBJ_JOINT, "front_wheel_spin");
    const int rw_jnt = mj_name2id(m, mjOBJ_JOINT, "rear_wheel_spin");
    require(crank_jnt >= 0 && fw_jnt >= 0 && rw_jnt >= 0, "spin joints required");

    cfg.efc_scan.grip_left_eq_id = mj_name2id(m, mjOBJ_EQUALITY, "connect_grip_left");
    cfg.efc_scan.grip_right_eq_id = mj_name2id(m, mjOBJ_EQUALITY, "connect_grip_right");
    cfg.efc_scan.saddle_eq_id = mj_name2id(m, mjOBJ_EQUALITY, "connect_saddle");
    cfg.efc_scan.foot_front_eq_id = mj_name2id(m, mjOBJ_EQUALITY, "connect_foot_front");
    cfg.efc_scan.foot_rear_eq_id = mj_name2id(m, mjOBJ_EQUALITY, "connect_foot_rear");
    cfg.efc_scan.frame_body_id = mj_name2id(m, mjOBJ_BODY, "frame");
    cfg.efc_scan.crank_dof = jnt_dofadr[to_index(crank_jnt)];
    cfg.efc_scan.front_wheel_dof = jnt_dofadr[to_index(fw_jnt)];
    cfg.efc_scan.rear_wheel_dof = jnt_dofadr[to_index(rw_jnt)];
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
        const auto ctrlrange =
            model_access::readonly_buffer(m->actuator_ctrlrange, 2 * m->nu, "actuator_ctrlrange");
        cfg.motor_ctrl_min_nm = static_cast<float>(ctrlrange[2 * to_index(motor_act) + 0]);
        cfg.motor_ctrl_max_nm = static_cast<float>(ctrlrange[2 * to_index(motor_act) + 1]);
    }

    const int accel_id = mj_name2id(m, mjOBJ_SENSOR, "sensor_frame_accel");
    const int gyro_id = mj_name2id(m, mjOBJ_SENSOR, "sensor_frame_gyro");
    cfg.accel_sensor_adr = (accel_id >= 0) ? sensor_adr[to_index(accel_id)] : -1;
    cfg.gyro_sensor_adr = (gyro_id >= 0) ? sensor_adr[to_index(gyro_id)] : -1;

    if (m->ntendon >= bike::physics_core::kCassetteCogCount) {
        const std::span<int> tid_span{cfg.shifting.cassette_tendon_ids};
        for (int c = 0; c < bike::physics_core::kCassetteCogCount; ++c) {
            const std::string tname = std::format("freehub_cog_{:02d}", c);
            const int tid = mj_name2id(m, mjOBJ_TENDON, tname.c_str());
            require(tid >= 0, "missing cassette tendon: " + tname);
            tid_span[to_index(c)] = tid;
        }
        cfg.efc_scan.freehub_tendon_id = tid_span[0];
    } else {
        int fh_tid = mj_name2id(m, mjOBJ_TENDON, "ideal_mid_drive_freehub");
        if (fh_tid < 0) {
            fh_tid = mj_name2id(m, mjOBJ_TENDON, "freehub");
        }
        cfg.efc_scan.freehub_tendon_id = fh_tid;
    }
    cfg.efc_scan.crank_clutch_tendon_id = mj_name2id(m, mjOBJ_TENDON, "crank_clutch");
    return cfg_ptr;
}

struct RidingInitKinematics {
    double speed_mps = 0.0;
    double crank_omega_rad_s = 0.0;
};

void build_riding_initial_state(
    const mjModel *m,
    const bike::gpu::GpuModelIndices &idx,
    const RidingInitKinematics &kin,
    DataPtr &out_data
) {
    const double speed_mps = kin.speed_mps;
    const double crank_omega_rad_s = kin.crank_omega_rad_s;
    DataPtr d(mj_makeData(m), &mj_deleteData);
    mj_resetData(m, d.get());

    const int nq = idx.nq;
    const int nv = idx.nv;
    const int rider_rx_jnt = mj_name2id(m, mjOBJ_JOINT, "rider_root_x");
    const int rider_rz_jnt = mj_name2id(m, mjOBJ_JOINT, "rider_root_z");
    require(rider_rx_jnt >= 0 && rider_rz_jnt >= 0, "welded.xml requires rider_root_x/z");

    const auto jnt_qposadr = model_access::readonly_buffer(m->jnt_qposadr, m->njnt, "jnt_qposadr");
    const auto jnt_dofadr = model_access::readonly_buffer(m->jnt_dofadr, m->njnt, "jnt_dofadr");
    const int rider_rx_qpos = jnt_qposadr[to_index(rider_rx_jnt)];
    const int rider_rz_qpos = jnt_qposadr[to_index(rider_rz_jnt)];
    const int rider_rx_dof = jnt_dofadr[to_index(rider_rx_jnt)];

    const auto d_qpos = model_access::mutable_buffer(d->qpos, nq, "qpos");
    const auto d_qvel = model_access::mutable_buffer(d->qvel, nv, "qvel");
    d_qpos[to_index(idx.root_x_qpos)] = 2.0;
    d_qpos[to_index(rider_rx_qpos)] = 2.0;
    mj_forward(m, d.get());

    const int f_body = idx.front_wheel_body;
    const int r_body = idx.rear_wheel_body;
    const int f_geom = bike::gpu::find_wheel_contact_geom(m, f_body, "front_contact", "geom_front_contact");
    const int r_geom = bike::gpu::find_wheel_contact_geom(m, r_body, "rear_contact", "geom_rear_contact");
    require(f_geom >= 0 && r_geom >= 0, "wheel contact geoms required");
    const auto geom_size = model_access::readonly_buffer(m->geom_size, 3 * m->ngeom, "geom_size");
    const auto d_xpos =
        model_access::readonly_buffer(d->xpos, 3LL * static_cast<mjtSize>(idx.nbody), "xpos");
    const double f_r = geom_size[3 * to_index(f_geom)];
    const double r_r = geom_size[3 * to_index(r_geom)];
    const double min_clear = std::min(
        d_xpos[3 * to_index(f_body) + 2] - f_r,
        d_xpos[3 * to_index(r_body) + 2] - r_r);
    const double dz = -0.015 - min_clear;
    d_qpos[to_index(idx.root_z_qpos)] += dz;
    d_qpos[to_index(rider_rz_qpos)] += dz;
    d_qvel[to_index(idx.root_x_dof)] = speed_mps;
    d_qvel[to_index(rider_rx_dof)] = speed_mps;
    d_qvel[to_index(idx.front_wheel_spin_dof)] = speed_mps / f_r;
    d_qvel[to_index(idx.rear_wheel_spin_dof)] = speed_mps / r_r;
    d_qvel[to_index(idx.crank_spin_dof)] = crank_omega_rad_s;
    if (idx.drive_shaft_spin_dof >= 0) {
        d_qvel[to_index(idx.drive_shaft_spin_dof)] = crank_omega_rad_s;
    }
    mj_forward(m, d.get());
    out_data = std::move(d);
}

struct CpuWorldBuffers {
    std::vector<float> qpos;
    std::vector<float> qvel;
    std::vector<float> xpos;
    std::vector<float> xmat;
    std::vector<float> xanchor;
    std::vector<float> xaxis;
    std::vector<float> qfrc;
    std::vector<float> ctrl;
    std::vector<float> tendon_range;
    std::vector<float> dof_frictionloss;
    std::vector<float> sensordata;
    std::vector<int> nefc;
    std::vector<int> efc_type;
    std::vector<int> efc_id;
    std::vector<float> efc_J;
    std::vector<float> efc_force;
    std::vector<int> nacon;
    std::vector<int> contact_worldid;
    std::vector<int> contact_geom;
    std::vector<int> contact_efc_address;
    std::vector<int> contact_dim;
};

CpuWorldBuffers snapshot_mj_to_float(const mjModel *m, const mjData *d) {
    CpuWorldBuffers b;
    const int nq = to_int(m->nq);
    const int nv = to_int(m->nv);
    const int nu = to_int(m->nu);
    const int nbody = to_int(m->nbody);
    const int njnt = to_int(m->njnt);
    const int ntendon = to_int(m->ntendon);
    const int nsd = to_int(m->nsensordata);
    const int nefc = d->nefc;
    const int ncon = d->ncon;

    b.qpos.resize(to_index(nq));
    b.qvel.resize(to_index(nv));
    b.xpos.resize(to_index(nbody) * 3ULL);
    b.xmat.resize(to_index(nbody) * 9ULL);
    b.xanchor.resize(to_index(njnt) * 3ULL);
    b.xaxis.resize(to_index(njnt) * 3ULL);
    b.qfrc.assign(to_index(nv), 0.0f);
    b.ctrl.assign(to_index(std::max(1, nu)), 0.0f);
    b.tendon_range.assign(to_index(std::max(1, 2 * ntendon)), 0.0f);
    b.dof_frictionloss.assign(to_index(nv), 0.0f);
    b.sensordata.assign(to_index(std::max(1, nsd)), 0.0f);

    const auto qp = model_access::readonly_buffer(d->qpos, nq, "qpos");
    const auto qv = model_access::readonly_buffer(d->qvel, nv, "qvel");
    const auto xp = model_access::readonly_buffer(d->xpos, 3LL * nbody, "xpos");
    const auto xm = model_access::readonly_buffer(d->xmat, 9LL * nbody, "xmat");
    const auto xa = model_access::readonly_buffer(d->xanchor, 3LL * njnt, "xanchor");
    const auto xx = model_access::readonly_buffer(d->xaxis, 3LL * njnt, "xaxis");
    const auto tr = model_access::readonly_buffer(m->tendon_range, 2LL * ntendon, "tendon_range");
    const auto dfl = model_access::readonly_buffer(m->dof_frictionloss, nv, "dof_frictionloss");
    const auto sd = model_access::readonly_buffer(d->sensordata, nsd, "sensordata");

    for (int i = 0; i < nq; ++i) b.qpos[to_index(i)] = static_cast<float>(qp[to_index(i)]);
    for (int i = 0; i < nv; ++i) b.qvel[to_index(i)] = static_cast<float>(qv[to_index(i)]);
    for (int i = 0; i < 3 * nbody; ++i) b.xpos[to_index(i)] = static_cast<float>(xp[to_index(i)]);
    for (int i = 0; i < 9 * nbody; ++i) b.xmat[to_index(i)] = static_cast<float>(xm[to_index(i)]);
    for (int i = 0; i < 3 * njnt; ++i) b.xanchor[to_index(i)] = static_cast<float>(xa[to_index(i)]);
    for (int i = 0; i < 3 * njnt; ++i) b.xaxis[to_index(i)] = static_cast<float>(xx[to_index(i)]);
    for (int i = 0; i < 2 * ntendon; ++i) b.tendon_range[to_index(i)] = static_cast<float>(tr[to_index(i)]);
    for (int i = 0; i < nv; ++i) b.dof_frictionloss[to_index(i)] = static_cast<float>(dfl[to_index(i)]);
    for (int i = 0; i < nsd; ++i) b.sensordata[to_index(i)] = static_cast<float>(sd[to_index(i)]);

    b.nefc = {nefc};
    if (nefc > 0) {
        b.efc_type.resize(to_index(nefc));
        b.efc_id.resize(to_index(nefc));
        b.efc_force.resize(to_index(nefc));
        b.efc_J.resize(to_index(nefc) * to_index(nv));
        const auto et = model_access::readonly_buffer(d->efc_type, nefc, "efc_type");
        const auto eid = model_access::readonly_buffer(d->efc_id, nefc, "efc_id");
        const auto ef = model_access::readonly_buffer(d->efc_force, nefc, "efc_force");
        const auto eJ =
            model_access::readonly_buffer(d->efc_J, static_cast<mjtSize>(nefc) * nv, "efc_J");
        for (int r = 0; r < nefc; ++r) {
            b.efc_type[to_index(r)] = et[to_index(r)];
            b.efc_id[to_index(r)] = eid[to_index(r)];
            b.efc_force[to_index(r)] = static_cast<float>(ef[to_index(r)]);
        }
        for (int i = 0; i < nefc * nv; ++i) {
            b.efc_J[to_index(i)] = static_cast<float>(eJ[to_index(i)]);
        }
    }

    b.nacon = {ncon};
    if (ncon > 0) {
        b.contact_worldid.assign(to_index(ncon), 0);
        b.contact_geom.resize(to_index(ncon) * 2ULL);
        b.contact_efc_address.resize(to_index(ncon) * 4ULL);
        b.contact_dim.resize(to_index(ncon));
        const auto cons = model_access::readonly_buffer(d->contact, ncon, "contact");
        for (int c = 0; c < ncon; ++c) {
            b.contact_geom[to_index(c) * 2ULL + 0ULL] = cons[to_index(c)].geom[0];
            b.contact_geom[to_index(c) * 2ULL + 1ULL] = cons[to_index(c)].geom[1];
            b.contact_dim[to_index(c)] = cons[to_index(c)].dim;
            b.contact_efc_address[to_index(c) * 4ULL + 0ULL] = cons[to_index(c)].efc_address;
            b.contact_efc_address[to_index(c) * 4ULL + 1ULL] = -1;
            b.contact_efc_address[to_index(c) * 4ULL + 2ULL] = -1;
            b.contact_efc_address[to_index(c) * 4ULL + 3ULL] = -1;
        }
    }
    return b;
}

void write_pre_solve_back_to_mj(
    const mjModel *m,
    const mjData *d,
    const CpuWorldBuffers &b
) {
    const int nv = to_int(m->nv);
    const int nu = to_int(m->nu);
    const int ntendon = to_int(m->ntendon);
    const auto qfrc = model_access::mutable_buffer(d->qfrc_applied, nv, "qfrc_applied");
    const auto ctrl = model_access::mutable_buffer(d->ctrl, nu, "ctrl");
    const auto tr = model_access::mutable_buffer(m->tendon_range, 2LL * ntendon, "tendon_range");
    const auto dfl = model_access::mutable_buffer(m->dof_frictionloss, nv, "dof_frictionloss");
    for (int i = 0; i < nv; ++i) qfrc[to_index(i)] = static_cast<mjtNum>(b.qfrc[to_index(i)]);
    for (int i = 0; i < nu; ++i) ctrl[to_index(i)] = static_cast<mjtNum>(b.ctrl[to_index(i)]);
    for (int i = 0; i < 2 * ntendon; ++i) tr[to_index(i)] = static_cast<mjtNum>(b.tendon_range[to_index(i)]);
    for (int i = 0; i < nv; ++i) dfl[to_index(i)] = static_cast<mjtNum>(b.dof_frictionloss[to_index(i)]);
}

void test_12tendon_steady_gear_and_shift(const TempSp3Models &models) {
    ModelPtr m2 = load_xml(models.xml2);
    ModelPtr m13 = load_xml(models.xml13);
    require(m2->ntendon == 2, "welded.xml must have ntendon == 2");
    require(m13->ntendon == 13, "model_12gears.xml must have ntendon == 13");
    require(m2->nq == m13->nq && m2->nv == m13->nv && m2->nu == m13->nu,
            "12-gear expansion must keep nq/nv/nu unchanged");

    const auto idx2 = resolve_model_indices(m2.get());
    const auto idx13 = resolve_model_indices(m13.get());
    const auto tt2 = build_tyre_terrain_from_xml(models.xml2, idx2);
    const auto tt13 = build_tyre_terrain_from_xml(models.xml13, idx13);

    double max_abs_dqacc = 0.0;
    double max_abs_dqpos = 0.0;

    for (int coast_mode = 0; coast_mode < 2; ++coast_mode) {
        const double speed = 5.0;
        const double crank_w = (coast_mode == 0) ? 7.5 : 0.0;
        const int nq2 = to_int(m2->nq);
        const int nv2 = to_int(m2->nv);
        const int nt2 = to_int(m2->ntendon);
        const int nsd2 = to_int(m2->nsensordata);
        const int nt13 = to_int(m13->ntendon);
        const int nv13 = to_int(m13->nv);
        const int nsd13 = to_int(m13->nsensordata);
        const auto t_adr2 = model_access::readonly_buffer(m2->tendon_adr, m2->ntendon, "t_adr2");
        const auto t_adr13 = model_access::readonly_buffer(m13->tendon_adr, m13->ntendon, "t_adr13");
        const auto wp13 = model_access::readonly_buffer(m13->wrap_prm, m13->nwrap, "wp13");
        const auto invw13 =
            model_access::readonly_buffer(m13->tendon_invweight0, m13->ntendon, "invw13");
        const auto wp2 = model_access::mutable_buffer(m2->wrap_prm, m2->nwrap, "wp2");
        const auto invw2 =
            model_access::mutable_buffer(m2->tendon_invweight0, m2->ntendon, "invw2");

        for (int cog_idx = 0; cog_idx < bike::physics_core::kCassetteCogCount; ++cog_idx) {
            const int teeth = bike::configuration::kCassette12sTeeth[to_index(cog_idx)];
            const auto cfg2 = build_sp3_config_for_model(m2.get());
            const auto cfg13 = build_sp3_config_for_model(m13.get());
            const int active_tid13 =
                std::span<const int>{cfg13->shifting.cassette_tendon_ids}[to_index(cog_idx)];
            wp2[to_index(t_adr2[to_index(idx2.freehub_tendon)])] =
                wp13[to_index(t_adr13[to_index(active_tid13)])];
            invw2[to_index(idx2.freehub_tendon)] = invw13[to_index(active_tid13)];
            cfg13->efc_scan.freehub_tendon_id = active_tid13;

            DataPtr d2(nullptr, &mj_deleteData);
            DataPtr d13(nullptr, &mj_deleteData);
            build_riding_initial_state(m2.get(), idx2, {.speed_mps = speed, .crank_omega_rad_s = crank_w}, d2);
            build_riding_initial_state(m13.get(), idx13, {.speed_mps = speed, .crank_omega_rad_s = crank_w}, d13);
            for (bike::gpu::GpuSp3Config *c : {cfg2.get(), cfg13.get()}) {
                c->shifting.initial_rear_teeth = teeth;
                c->shifting.enabled = 0;
                if (coast_mode == 1) {
                    c->effort_mode = 0;
                    c->caller_human_torque_nm = 0.0f;
                    c->caller_motor_torque_nm = 0.0f;
                }
            }

            const auto st2 = std::make_unique<bike::gpu::GpuSp3WorldState>();
            const auto st13 = std::make_unique<bike::gpu::GpuSp3WorldState>();
            const auto ws2 = std::make_unique<bike::gpu::GpuWorldState>();
            const auto ws13 = std::make_unique<bike::gpu::GpuWorldState>();

            CpuWorldBuffers b2 = snapshot_mj_to_float(m2.get(), d2.get());
            CpuWorldBuffers b13 = snapshot_mj_to_float(m13.get(), d13.get());

            require(bike_gpu_launch_sp3_init(
                        nullptr, 1, &idx2, cfg2.get(), 0, st2.get(), ws2.get(),
                        b2.qpos.data(), 0.0f, nt2, b2.tendon_range.data(),
                        2 * nt2) == 0,
                    "sp3 init 2-tendon failed");
            require(bike_gpu_launch_sp3_init(
                        nullptr, 1, &idx13, cfg13.get(), 0, st13.get(), ws13.get(),
                        b13.qpos.data(), 0.0f, nt13, b13.tendon_range.data(),
                        2 * nt13) == 0,
                    "sp3 init 13-tendon failed");

            // Check that inactive 11 cogs are unconstrained (+1e12f)
            const std::span<const int> tid_span{cfg13->shifting.cassette_tendon_ids};
            for (int k = 0; k < bike::physics_core::kCassetteCogCount; ++k) {
                const int tid = tid_span[to_index(k)];
                const float hi = b13.tendon_range[to_index(2 * tid + 1)];
                if (k == cog_idx) {
                    require(hi == b2.tendon_range[to_index(2 * idx2.freehub_tendon + 1)],
                            "active cassette cog upper bound mismatch at init");
                } else {
                    require(hi == 1e12f, "inactive cassette cog upper bound must be +1e12f");
                }
            }

            for (int step = 0; step < 8; ++step) {
                b2 = snapshot_mj_to_float(m2.get(), d2.get());
                b13 = snapshot_mj_to_float(m13.get(), d13.get());

                require(bike_gpu_launch_sp3_pre_solve(
                            nullptr, 1, kDtS, &idx2, &tt2->tyre, &tt2->terrain_desc,
                            tt2->terrain_f32.data(), tt2->terrain_i32.data(),
                            cfg2.get(), 0, st2.get(), ws2.get(),
                            b2.qpos.data(), b2.qvel.data(), b2.xpos.data(), b2.xmat.data(),
                            b2.xanchor.data(), b2.xaxis.data(), b2.qfrc.data(), b2.ctrl.data(),
                            nt2, b2.tendon_range.data(), 2 * nt2,
                            b2.dof_frictionloss.data(), nv2) == 0,
                        "sp3 pre_solve 2-tendon failed");
                require(bike_gpu_launch_sp3_pre_solve(
                            nullptr, 1, kDtS, &idx13, &tt13->tyre, &tt13->terrain_desc,
                            tt13->terrain_f32.data(), tt13->terrain_i32.data(),
                            cfg13.get(), 0, st13.get(), ws13.get(),
                            b13.qpos.data(), b13.qvel.data(), b13.xpos.data(), b13.xmat.data(),
                            b13.xanchor.data(), b13.xaxis.data(), b13.qfrc.data(), b13.ctrl.data(),
                            nt13, b13.tendon_range.data(), 2 * nt13,
                            b13.dof_frictionloss.data(), nv13) == 0,
                        "sp3 pre_solve 13-tendon failed");

                write_pre_solve_back_to_mj(m2.get(), d2.get(), b2);
                write_pre_solve_back_to_mj(m13.get(), d13.get(), b13);
                mj_step(m2.get(), d2.get());
                mj_step(m13.get(), d13.get());

                const auto qa2 = model_access::readonly_buffer(d2->qacc, nv2, "qacc2");
                const auto qa13 = model_access::readonly_buffer(d13->qacc, nv13, "qacc13");
                const auto qp2 = model_access::readonly_buffer(d2->qpos, nq2, "qpos2");
                const auto qp13 = model_access::readonly_buffer(d13->qpos, nq2, "qpos13");
                for (int i = 0; i < nv2; ++i) {
                    max_abs_dqacc = std::max(max_abs_dqacc, std::abs(qa2[to_index(i)] - qa13[to_index(i)]));
                }
                for (int i = 0; i < nq2; ++i) {
                    max_abs_dqpos = std::max(max_abs_dqpos, std::abs(qp2[to_index(i)] - qp13[to_index(i)]));
                }

                b2 = snapshot_mj_to_float(m2.get(), d2.get());
                b13 = snapshot_mj_to_float(m13.get(), d13.get());
                require(bike_gpu_launch_sp3_post_solve(
                            nullptr, 1, kDtS, &idx2, cfg2.get(), 0, st2.get(),
                            b2.qpos.data(), b2.qvel.data(), b2.xmat.data(),
                            nsd2, b2.sensordata.data(),
                            b2.nefc.data(), std::max(1, d2->nefc), nv2,
                            b2.efc_type.data(), b2.efc_id.data(), b2.efc_J.data(), b2.efc_force.data(),
                            b2.nacon.data(), std::max(1, d2->ncon), 4,
                            b2.contact_worldid.data(), b2.contact_geom.data(),
                            b2.contact_efc_address.data(), b2.contact_dim.data()) == 0,
                        "sp3 post_solve 2-tendon failed");
                require(bike_gpu_launch_sp3_post_solve(
                            nullptr, 1, kDtS, &idx13, cfg13.get(), 0, st13.get(),
                            b13.qpos.data(), b13.qvel.data(), b13.xmat.data(),
                            nsd13, b13.sensordata.data(),
                            b13.nefc.data(), std::max(1, d13->nefc), nv13,
                            b13.efc_type.data(), b13.efc_id.data(), b13.efc_J.data(), b13.efc_force.data(),
                            b13.nacon.data(), std::max(1, d13->ncon), 4,
                            b13.contact_worldid.data(), b13.contact_geom.data(),
                            b13.contact_efc_address.data(), b13.contact_dim.data()) == 0,
                        "sp3 post_solve 13-tendon failed");
                require(st2->last_efc.freehub_tendon_force_n == st13->last_efc.freehub_tendon_force_n,
                        "12-tendon vs 2-tendon freehub_tendon_force_n mismatch");
            }
        }
    }
    std::cout << "[test_12tendon_steady_gear_and_shift] max|dqacc|=" << max_abs_dqacc
              << " max|dqpos|=" << max_abs_dqpos << '\n';
    require(max_abs_dqacc == 0.0, "12-tendon vs 2-tendon steady gear qacc diff must be 0.0");
    require(max_abs_dqpos == 0.0, "12-tendon vs 2-tendon steady gear qpos diff must be 0.0");

    // Discrete gear shift + ratchet_shift_ratio_bound + coasting ratchet_prepare_bound
    const int nv13 = to_int(m13->nv);
    const int nt13 = to_int(m13->ntendon);
    const int nsd13 = to_int(m13->nsensordata);
    DataPtr d_sh(nullptr, &mj_deleteData);
    build_riding_initial_state(m13.get(), idx13, {.speed_mps = 6.0, .crank_omega_rad_s = 14.0}, d_sh);
    const auto cfg_sh = build_sp3_config_for_model(m13.get());
    cfg_sh->intent.enabled = 0;
    cfg_sh->caller_human_torque_nm = 25.0f;
    cfg_sh->shifting.enabled = 1;
    cfg_sh->shifting.initial_rear_teeth = 24;
    cfg_sh->shifting.shift_cooldown_ticks = 2;
    cfg_sh->shifting.shift_cut_duration_ticks = 1;
    cfg_sh->shifting.cadence_smoothing_tau_s = 0.001f;

    const auto st_sh = std::make_unique<bike::gpu::GpuSp3WorldState>();
    const auto ws_sh = std::make_unique<bike::gpu::GpuWorldState>();
    CpuWorldBuffers b_sh = snapshot_mj_to_float(m13.get(), d_sh.get());
    require(bike_gpu_launch_sp3_init(
                nullptr, 1, &idx13, cfg_sh.get(), 0, st_sh.get(), ws_sh.get(),
                b_sh.qpos.data(), 0.0f, nt13, b_sh.tendon_range.data(),
                2 * nt13) == 0,
            "sp3 shift init failed");

    const int initial_cog = st_sh->shifting.active_cog_idx;
    bool observed_shift = false;
    for (int step = 0; step < 20; ++step) {
        b_sh = snapshot_mj_to_float(m13.get(), d_sh.get());
        require(bike_gpu_launch_sp3_pre_solve(
                    nullptr, 1, kDtS, &idx13, &tt13->tyre, &tt13->terrain_desc,
                    tt13->terrain_f32.data(), tt13->terrain_i32.data(),
                    cfg_sh.get(), 0, st_sh.get(), ws_sh.get(),
                    b_sh.qpos.data(), b_sh.qvel.data(), b_sh.xpos.data(), b_sh.xmat.data(),
                    b_sh.xanchor.data(), b_sh.xaxis.data(), b_sh.qfrc.data(), b_sh.ctrl.data(),
                    nt13, b_sh.tendon_range.data(), 2 * nt13,
                    b_sh.dof_frictionloss.data(), nv13) == 0,
                "sp3 shift pre_solve failed");
        if (st_sh->shifting.shift_count > 0 && st_sh->shifting.active_cog_idx != initial_cog) {
            observed_shift = true;
            const std::span<const int> tid_span{cfg_sh->shifting.cassette_tendon_ids};
            for (int k = 0; k < bike::physics_core::kCassetteCogCount; ++k) {
                const int tid = tid_span[to_index(k)];
                const float hi = b_sh.tendon_range[to_index(2 * tid + 1)];
                if (k == st_sh->shifting.active_cog_idx) {
                    require(std::isfinite(hi) && hi < 1e11f, "shifted active cog bound must be finite");
                } else {
                    require(hi == 1e12f, "non-active cog after shift must be +1e12f");
                }
            }
        }
        write_pre_solve_back_to_mj(m13.get(), d_sh.get(), b_sh);
        mj_step(m13.get(), d_sh.get());
        b_sh = snapshot_mj_to_float(m13.get(), d_sh.get());
        require(bike_gpu_launch_sp3_post_solve(
                    nullptr, 1, kDtS, &idx13, cfg_sh.get(), 0, st_sh.get(),
                    b_sh.qpos.data(), b_sh.qvel.data(), b_sh.xmat.data(),
                    nsd13, b_sh.sensordata.data(),
                    b_sh.nefc.data(), std::max(1, d_sh->nefc), nv13,
                    b_sh.efc_type.data(), b_sh.efc_id.data(), b_sh.efc_J.data(), b_sh.efc_force.data(),
                    b_sh.nacon.data(), std::max(1, d_sh->ncon), 4,
                    b_sh.contact_worldid.data(), b_sh.contact_geom.data(),
                    b_sh.contact_efc_address.data(), b_sh.contact_dim.data()) == 0,
                "sp3 shift post_solve failed");
    }
    require(observed_shift, "expected autonomous cadence shift at 134 rpm");

    // Coasting ratchet_prepare_bound test: freeze crank while rear wheel advances
    const float bound_before_coast = st_sh->freehub.boundary;
    const auto qp_m = model_access::mutable_buffer(d_sh->qpos, m13->nq, "qpos_coast");
    qp_m[to_index(idx13.rear_wheel_spin_qpos)] -= 4.0; // wheel spins backwards relative to boundary sign check
    const double rw_step = qp_m[to_index(idx13.rear_wheel_spin_qpos)];
    qp_m[to_index(idx13.rear_wheel_spin_qpos)] = rw_step + 8.0; // advance rear wheel +8 rad while ds fixed
    cfg_sh->shifting.enabled = 0;
    cfg_sh->caller_human_torque_nm = 0.0f;
    b_sh = snapshot_mj_to_float(m13.get(), d_sh.get());
    require(bike_gpu_launch_sp3_pre_solve(
                nullptr, 1, kDtS, &idx13, &tt13->tyre, &tt13->terrain_desc,
                tt13->terrain_f32.data(), tt13->terrain_i32.data(),
                cfg_sh.get(), 0, st_sh.get(), ws_sh.get(),
                b_sh.qpos.data(), b_sh.qvel.data(), b_sh.xpos.data(), b_sh.xmat.data(),
                b_sh.xanchor.data(), b_sh.xaxis.data(), b_sh.qfrc.data(), b_sh.ctrl.data(),
                nt13, b_sh.tendon_range.data(), 2 * nt13,
                b_sh.dof_frictionloss.data(), nv13) == 0,
            "coasting pre_solve failed");
    const float ds_q_now = (idx13.drive_shaft_spin_qpos >= 0)
        ? b_sh.qpos[to_index(idx13.drive_shaft_spin_qpos)]
        : b_sh.qpos[to_index(idx13.crank_spin_qpos)];
    const float rw_q_now = b_sh.qpos[to_index(idx13.rear_wheel_spin_qpos)];
    const float ratio_now = bike::physics_core::shifting_gear_ratio<float>(&cfg_sh->shifting, &st_sh->shifting);
    require(st_sh->freehub.boundary <= bound_before_coast, "freehub ratchet_prepare_bound updated while coasting");
    require((ratio_now * ds_q_now - rw_q_now) <= st_sh->freehub.boundary + 1e-5f,
            "freehub boundary violated after coasting advance");
}

void test_post_solve_efc_reaction_parity(const TempSp3Models &models) {
    ModelPtr m13 = load_xml(models.xml13);
    const auto idx13 = resolve_model_indices(m13.get());
    const auto tt13 = build_tyre_terrain_from_xml(models.xml13, idx13);
    auto cfg13 = build_sp3_config_for_model(m13.get());
    const int nv13 = to_int(m13->nv);
    const int nt13 = to_int(m13->ntendon);
    const int nsd13 = to_int(m13->nsensordata);

    DataPtr d(nullptr, &mj_deleteData);
    build_riding_initial_state(m13.get(), idx13, {.speed_mps = 4.5, .crank_omega_rad_s = 8.0}, d);

    const auto st = std::make_unique<bike::gpu::GpuSp3WorldState>();
    const auto ws = std::make_unique<bike::gpu::GpuWorldState>();
    CpuWorldBuffers b = snapshot_mj_to_float(m13.get(), d.get());
    require(bike_gpu_launch_sp3_init(
                nullptr, 1, &idx13, cfg13.get(), 0, st.get(), ws.get(),
                b.qpos.data(), 0.0f, nt13, b.tendon_range.data(),
                2 * nt13) == 0,
            "efc parity init failed");

    for (int step = 0; step < 40; ++step) {
        b = snapshot_mj_to_float(m13.get(), d.get());
        require(bike_gpu_launch_sp3_pre_solve(
                    nullptr, 1, kDtS, &idx13, &tt13->tyre, &tt13->terrain_desc,
                    tt13->terrain_f32.data(), tt13->terrain_i32.data(),
                    cfg13.get(), 0, st.get(), ws.get(),
                    b.qpos.data(), b.qvel.data(), b.xpos.data(), b.xmat.data(),
                    b.xanchor.data(), b.xaxis.data(), b.qfrc.data(), b.ctrl.data(),
                    nt13, b.tendon_range.data(), 2 * nt13,
                    b.dof_frictionloss.data(), nv13) == 0,
                "efc parity pre_solve failed");
        write_pre_solve_back_to_mj(m13.get(), d.get(), b);
        mj_step(m13.get(), d.get());
        b = snapshot_mj_to_float(m13.get(), d.get());
        require(bike_gpu_launch_sp3_post_solve(
                    nullptr, 1, kDtS, &idx13, cfg13.get(), 0, st.get(),
                    b.qpos.data(), b.qvel.data(), b.xmat.data(),
                    nsd13, b.sensordata.data(),
                    b.nefc.data(), std::max(1, d->nefc), nv13,
                    b.efc_type.data(), b.efc_id.data(), b.efc_J.data(), b.efc_force.data(),
                    b.nacon.data(), std::max(1, d->ncon), 4,
                    b.contact_worldid.data(), b.contact_geom.data(),
                    b.contact_efc_address.data(), b.contact_dim.data()) == 0,
                "efc parity post_solve failed");
    }

    // Compare against CPU mj_mulJacTVec on foot_front + foot_rear equality rows
    mj_forward(m13.get(), d.get());
    b = snapshot_mj_to_float(m13.get(), d.get());
    require(bike_gpu_launch_sp3_post_solve(
                nullptr, 1, kDtS, &idx13, cfg13.get(), 0, st.get(),
                b.qpos.data(), b.qvel.data(), b.xmat.data(),
                nsd13, b.sensordata.data(),
                b.nefc.data(), std::max(1, d->nefc), nv13,
                b.efc_type.data(), b.efc_id.data(), b.efc_J.data(), b.efc_force.data(),
                b.nacon.data(), std::max(1, d->ncon), 4,
                b.contact_worldid.data(), b.contact_geom.data(),
                b.contact_efc_address.data(), b.contact_dim.data()) == 0,
            "efc parity final post_solve failed");

    const int nefc = d->nefc;
    require(nefc > 0, "expected active constraints for efc parity");
    const auto et = model_access::readonly_buffer(d->efc_type, nefc, "efc_type");
    const auto eid = model_access::readonly_buffer(d->efc_id, nefc, "efc_id");
    const auto ef = model_access::readonly_buffer(d->efc_force, nefc, "efc_force");
    const auto xm = model_access::readonly_buffer(d->xmat, 9 * m13->nbody, "xmat");

    std::vector<mjtNum> masked_force(to_index(nefc), 0.0);
    std::vector<mjtNum> qfrc_feet(to_index(m13->nv), 0.0);
    std::array<double, 3> gl_f{};
    std::array<double, 3> gr_f{};
    int gl_c = 0;
    int gr_c = 0;
    for (int r = 0; r < nefc; ++r) {
        const auto ru = to_index(r);
        if (et[ru] == mjCNSTR_EQUALITY) {
            if (eid[ru] == cfg13->efc_scan.foot_front_eq_id ||
                eid[ru] == cfg13->efc_scan.foot_rear_eq_id) {
                masked_force[ru] = ef[ru];
            } else if (eid[ru] == cfg13->efc_scan.grip_left_eq_id && gl_c < 3) {
                gl_f[to_index(gl_c++)] = static_cast<double>(ef[ru]);
            } else if (eid[ru] == cfg13->efc_scan.grip_right_eq_id && gr_c < 3) {
                gr_f[to_index(gr_c++)] = static_cast<double>(ef[ru]);
            }
        }
    }
    mj_mulJacTVec(m13.get(), d.get(), qfrc_feet.data(), masked_force.data());
    const double cpu_crank_tq = static_cast<double>(qfrc_feet[to_index(idx13.crank_spin_dof)]);
    const int fb = 9 * cfg13->efc_scan.frame_body_id;
    const double up0 = static_cast<double>(xm[to_index(fb + 2)]);
    const double up1 = static_cast<double>(xm[to_index(fb + 5)]);
    const double up2 = static_cast<double>(xm[to_index(fb + 8)]);
    const double cpu_hand_press =
        (gl_c == 3 ? (gl_f[0] * up0 + gl_f[1] * up1 + gl_f[2] * up2) : 0.0) +
        (gr_c == 3 ? (gr_f[0] * up0 + gr_f[1] * up1 + gr_f[2] * up2) : 0.0);

    const double err_crank = std::abs(static_cast<double>(st->last_efc.delivered_crank_torque_nm) - cpu_crank_tq);
    const double err_hand = std::abs(static_cast<double>(st->last_efc.hand_press_n) - cpu_hand_press);
    std::cout << "[test_post_solve_efc_reaction_parity] crank_tq=" << cpu_crank_tq
              << " err=" << err_crank << " hand_press=" << cpu_hand_press
              << " err=" << err_hand << '\n';
    require(err_crank <= 5.0e-4, "delivered_crank_torque_nm mismatch vs mj_mulJacTVec");
    require(err_hand <= 5.0e-4, "hand_press_n mismatch vs CPU grip reaction");
}

void test_2pi_epoch_rebasing_invariance(const TempSp3Models &models) {
    ModelPtr m13 = load_xml(models.xml13);
    const auto idx13 = resolve_model_indices(m13.get());
    const auto tt13 = build_tyre_terrain_from_xml(models.xml13, idx13);
    const auto cfg = build_sp3_config_for_model(m13.get());
    const int nv13 = to_int(m13->nv);
    const int nu13 = to_int(m13->nu);
    const int nt13 = to_int(m13->ntendon);
    const int nsd13 = to_int(m13->nsensordata);

    DataPtr d(nullptr, &mj_deleteData);
    build_riding_initial_state(m13.get(), idx13, {.speed_mps = 5.0, .crank_omega_rad_s = 8.0}, d);

    const auto st_ref = std::make_unique<bike::gpu::GpuSp3WorldState>();
    const auto ws_ref = std::make_unique<bike::gpu::GpuWorldState>();
    CpuWorldBuffers b_ref = snapshot_mj_to_float(m13.get(), d.get());
    require(bike_gpu_launch_sp3_init(
                nullptr, 1, &idx13, cfg.get(), 0, st_ref.get(), ws_ref.get(),
                b_ref.qpos.data(), 0.0f, nt13, b_ref.tendon_range.data(),
                2 * nt13) == 0,
            "rebase init failed");

    // Step twice (pre + post) and run pre_solve of step 2 to populate latches & monitors cleanly
    for (int step = 0; step < 2; ++step) {
        require(bike_gpu_launch_sp3_pre_solve(
                    nullptr, 1, kDtS, &idx13, &tt13->tyre, &tt13->terrain_desc,
                    tt13->terrain_f32.data(), tt13->terrain_i32.data(),
                    cfg.get(), 0, st_ref.get(), ws_ref.get(),
                    b_ref.qpos.data(), b_ref.qvel.data(), b_ref.xpos.data(), b_ref.xmat.data(),
                    b_ref.xanchor.data(), b_ref.xaxis.data(), b_ref.qfrc.data(), b_ref.ctrl.data(),
                    nt13, b_ref.tendon_range.data(), 2 * nt13,
                    b_ref.dof_frictionloss.data(), nv13) == 0,
                "rebase warmup pre failed");
        require(bike_gpu_launch_sp3_post_solve(
                    nullptr, 1, kDtS, &idx13, cfg.get(), 0, st_ref.get(),
                    b_ref.qpos.data(), b_ref.qvel.data(), b_ref.xmat.data(),
                    nsd13, b_ref.sensordata.data(),
                    b_ref.nefc.data(), std::max(1, d->nefc), nv13,
                    b_ref.efc_type.data(), b_ref.efc_id.data(), b_ref.efc_J.data(), b_ref.efc_force.data(),
                    b_ref.nacon.data(), std::max(1, d->ncon), 4,
                    b_ref.contact_worldid.data(), b_ref.contact_geom.data(),
                    b_ref.contact_efc_address.data(), b_ref.contact_dim.data()) == 0,
                "rebase warmup post failed");
    }
    require(bike_gpu_launch_sp3_pre_solve(
                nullptr, 1, kDtS, &idx13, &tt13->tyre, &tt13->terrain_desc,
                tt13->terrain_f32.data(), tt13->terrain_i32.data(),
                cfg.get(), 0, st_ref.get(), ws_ref.get(),
                b_ref.qpos.data(), b_ref.qvel.data(), b_ref.xpos.data(), b_ref.xmat.data(),
                b_ref.xanchor.data(), b_ref.xaxis.data(), b_ref.qfrc.data(), b_ref.ctrl.data(),
                nt13, b_ref.tendon_range.data(), 2 * nt13,
                b_ref.dof_frictionloss.data(), nv13) == 0,
            "rebase step 2 pre failed");

    // Clone reference and shift wheel/crank by +200*2*pi and drive_shaft by +195*2*pi
    // so crank_clutch boundary re-basing is genuinely non-zero and tested.
    const float kTwoPi = 6.28318530717958647692f;
    const float shift_rad = 200.0f * kTwoPi;
    const bool has_ds = (idx13.drive_shaft_spin_qpos >= 0);
    const float shift_ds_rad = has_ds ? (195.0f * kTwoPi) : shift_rad;
    const auto st_big = std::make_unique<bike::gpu::GpuSp3WorldState>(*st_ref);
    const auto ws_big = std::make_unique<bike::gpu::GpuWorldState>(*ws_ref);
    CpuWorldBuffers b_big = b_ref;

    b_big.qpos[to_index(idx13.front_wheel_spin_qpos)] += shift_rad;
    b_big.qpos[to_index(idx13.rear_wheel_spin_qpos)] += shift_rad;
    b_big.qpos[to_index(idx13.crank_spin_qpos)] += shift_rad;
    if (has_ds) {
        b_big.qpos[to_index(idx13.drive_shaft_spin_qpos)] += shift_ds_rad;
    }
    const float shift_fh_driver = has_ds ? shift_ds_rad : shift_rad;
    st_big->freehub.boundary += st_big->freehub.ratio * shift_fh_driver - shift_rad;
    if (has_ds) {
        st_big->crank_clutch.boundary += shift_rad - shift_ds_rad;
    }
    st_big->sp3.monitors.brake.latch_front_rad += shift_rad;
    st_big->sp3.monitors.brake.latch_rear_rad += shift_rad;
    st_big->pedaling.target_phase_rad += shift_rad;
    st_big->pedaling.reposition.goal_rad += shift_rad;

    require(bike_gpu_launch_sp3_post_solve(
                nullptr, 1, kDtS, &idx13, cfg.get(), 0, st_big.get(),
                b_big.qpos.data(), b_big.qvel.data(), b_big.xmat.data(),
                nsd13, b_big.sensordata.data(),
                b_big.nefc.data(), std::max(1, d->nefc), nv13,
                b_big.efc_type.data(), b_big.efc_id.data(), b_big.efc_J.data(), b_big.efc_force.data(),
                b_big.nacon.data(), std::max(1, d->ncon), 4,
                b_big.contact_worldid.data(), b_big.contact_geom.data(),
                b_big.contact_efc_address.data(), b_big.contact_dim.data()) == 0,
            "rebase trigger post_solve failed");

    require(std::abs(b_big.qpos[to_index(idx13.front_wheel_spin_qpos)]) <= kTwoPi,
            "front_wheel qpos not wrapped by 2*pi rebase");
    require(std::abs(b_big.qpos[to_index(idx13.rear_wheel_spin_qpos)]) <= kTwoPi,
            "rear_wheel qpos not wrapped by 2*pi rebase");
    require(std::abs(b_big.qpos[to_index(idx13.crank_spin_qpos)]) <= kTwoPi,
            "crank qpos not wrapped by 2*pi rebase");
    if (has_ds) {
        require(std::abs(b_big.qpos[to_index(idx13.drive_shaft_spin_qpos)]) <= kTwoPi,
                "drive_shaft qpos not wrapped by 2*pi rebase");
    }

    // Advance st_ref one post_solve too so both match on next pre_solve
    require(bike_gpu_launch_sp3_post_solve(
                nullptr, 1, kDtS, &idx13, cfg.get(), 0, st_ref.get(),
                b_ref.qpos.data(), b_ref.qvel.data(), b_ref.xmat.data(),
                nsd13, b_ref.sensordata.data(),
                b_ref.nefc.data(), std::max(1, d->nefc), nv13,
                b_ref.efc_type.data(), b_ref.efc_id.data(), b_ref.efc_J.data(), b_ref.efc_force.data(),
                b_ref.nacon.data(), std::max(1, d->ncon), 4,
                b_ref.contact_worldid.data(), b_ref.contact_geom.data(),
                b_ref.contact_efc_address.data(), b_ref.contact_dim.data()) == 0,
            "ref post_solve failed");

    require(bike_gpu_launch_sp3_pre_solve(
                nullptr, 1, kDtS, &idx13, &tt13->tyre, &tt13->terrain_desc,
                tt13->terrain_f32.data(), tt13->terrain_i32.data(),
                cfg.get(), 0, st_ref.get(), ws_ref.get(),
                b_ref.qpos.data(), b_ref.qvel.data(), b_ref.xpos.data(), b_ref.xmat.data(),
                b_ref.xanchor.data(), b_ref.xaxis.data(), b_ref.qfrc.data(), b_ref.ctrl.data(),
                nt13, b_ref.tendon_range.data(), 2 * nt13,
                b_ref.dof_frictionloss.data(), nv13) == 0,
            "ref next pre_solve failed");
    require(bike_gpu_launch_sp3_pre_solve(
                nullptr, 1, kDtS, &idx13, &tt13->tyre, &tt13->terrain_desc,
                tt13->terrain_f32.data(), tt13->terrain_i32.data(),
                cfg.get(), 0, st_big.get(), ws_big.get(),
                b_big.qpos.data(), b_big.qvel.data(), b_big.xpos.data(), b_big.xmat.data(),
                b_big.xanchor.data(), b_big.xaxis.data(), b_big.qfrc.data(), b_big.ctrl.data(),
                nt13, b_big.tendon_range.data(), 2 * nt13,
                b_big.dof_frictionloss.data(), nv13) == 0,
            "big next pre_solve failed");

    double max_dqfrc = 0.0;
    for (int i = 0; i < nv13; ++i) {
        const auto iu = to_index(i);
        require(std::isfinite(b_big.qfrc[iu]), "nonfinite qfrc after 2*pi rebase");
        max_dqfrc = std::max(max_dqfrc, std::abs(static_cast<double>(b_ref.qfrc[iu] - b_big.qfrc[iu])));
    }
    double max_dctrl = 0.0;
    for (int i = 0; i < nu13; ++i) {
        const auto iu = to_index(i);
        require(std::isfinite(b_big.ctrl[iu]), "nonfinite ctrl after 2*pi rebase");
        max_dctrl = std::max(max_dctrl, std::abs(static_cast<double>(b_ref.ctrl[iu] - b_big.ctrl[iu])));
    }
    std::cout << "[test_2pi_epoch_rebasing_invariance] max|dqfrc|=" << max_dqfrc
              << " max|dctrl|=" << max_dctrl << '\n';
    require(max_dqfrc <= 5e-4, "2*pi rebase altered qfrc beyond tolerance");
    require(max_dctrl <= 5e-4, "2*pi rebase altered ctrl beyond tolerance");
}

void test_host_vs_metal_and_multiworld_isolation(const TempSp3Models &models) {
    ModelPtr m13 = load_xml(models.xml13);
    const auto idx = resolve_model_indices(m13.get());
    const auto tt = build_tyre_terrain_from_xml(models.xml13, idx);
    const auto base_cfg = build_sp3_config_for_model(m13.get());

    constexpr int kNworld = 64;
    const int nq = idx.nq;
    const int nv = idx.nv;
    const int nu = idx.nu;
    const int nbody = idx.nbody;
    const int njnt = to_int(m13->njnt);
    const int ntendon = to_int(m13->ntendon);
    const int tr_stride = 2 * ntendon;
    const int nsd = to_int(m13->nsensordata);

    DataPtr d0(nullptr, &mj_deleteData);
    build_riding_initial_state(m13.get(), idx, {.speed_mps = 5.0, .crank_omega_rad_s = 8.0}, d0);
    mj_step(m13.get(), d0.get());
    const CpuWorldBuffers proto = snapshot_mj_to_float(m13.get(), d0.get());
    const int njmax = std::max(1, d0->nefc);
    const int ncon_w = std::max(1, d0->ncon);
    const std::size_t kNwU = to_index(kNworld);
    const std::size_t nq_u = to_index(nq);
    const std::size_t nv_u = to_index(nv);
    const std::size_t nbody3_u = to_index(nbody) * 3ULL;
    const std::size_t nbody9_u = to_index(nbody) * 9ULL;
    const std::size_t njnt3_u = to_index(njnt) * 3ULL;
    const std::size_t tr_u = to_index(tr_stride);
    const std::size_t nsd_u = to_index(nsd);
    const std::size_t njmax_u = to_index(njmax);
    const std::size_t eJ_world_u = njmax_u * nv_u;
    const int nconmax = kNworld * ncon_w;

    std::vector<bike::gpu::GpuSp3Config> cfgs(kNwU, *base_cfg);
    for (int w = 0; w < kNworld; ++w) {
        auto &c = cfgs[to_index(w)];
        c.suspension.params.air_spring.gauge_pressure_psi *= (0.85f + 0.005f * static_cast<float>(w));
        c.tire_stiff_scale = 0.85f + 0.005f * static_cast<float>(w);
        c.shifting.initial_rear_teeth =
            bike::configuration::kCassette12sTeeth[to_index(w % bike::physics_core::kCassetteCogCount)];
        c.caller_front_brake = (w % 5 == 0) ? 0.25f : 0.0f;
        bike::gpu::gpu_init_suspension_preloads(c.suspension);
    }

    std::vector<float> qpos_batch(kNwU * nq_u);
    std::vector<float> qvel_batch(kNwU * nv_u);
    std::vector<float> xpos_batch(kNwU * nbody3_u);
    std::vector<float> xmat_batch(kNwU * nbody9_u);
    std::vector<float> xanchor_batch(kNwU * njnt3_u);
    std::vector<float> xaxis_batch(kNwU * njnt3_u);
    const std::vector<float> qfrc_batch(kNwU * nv_u, 0.0f);
    const std::vector<float> ctrl_batch(kNwU * to_index(nu), 0.0f);
    std::vector<float> tr_batch(kNwU * tr_u, 0.0f);
    std::vector<float> fl_batch(kNwU * nv_u, 0.0f);
    std::vector<float> sd_batch(kNwU * nsd_u, 0.0f);
    std::vector<int> nefc_batch(kNwU, d0->nefc);
    std::vector<int> et_batch(kNwU * njmax_u, 0);
    std::vector<int> eid_batch(kNwU * njmax_u, 0);
    std::vector<float> eJ_batch(kNwU * eJ_world_u, 0.0f);
    std::vector<float> ef_batch(kNwU * njmax_u, 0.0f);
    std::vector<int> nacon_total = {kNworld * d0->ncon};
    std::vector<int> c_wid_batch(to_index(nconmax), 0);
    std::vector<int> c_geom_batch(to_index(nconmax) * 2ULL, 0);
    std::vector<int> c_adr_batch(to_index(nconmax) * 4ULL, -1);
    std::vector<int> c_dim_batch(to_index(nconmax), 1);

    for (int w = 0; w < kNworld; ++w) {
        const std::size_t wu = to_index(w);
        std::ranges::copy(proto.qpos, qpos_batch.begin() + static_cast<std::ptrdiff_t>(wu * nq_u));
        std::ranges::copy(proto.qvel, qvel_batch.begin() + static_cast<std::ptrdiff_t>(wu * nv_u));
        // Heterogeneous phase offset per world
        qpos_batch[wu * nq_u + to_index(idx.crank_spin_qpos)] += 0.03f * static_cast<float>(w);
        qvel_batch[wu * nv_u + to_index(idx.crank_spin_dof)] += 0.1f * static_cast<float>(w % 7);
        std::ranges::copy(proto.xpos, xpos_batch.begin() + static_cast<std::ptrdiff_t>(wu * nbody3_u));
        std::ranges::copy(proto.xmat, xmat_batch.begin() + static_cast<std::ptrdiff_t>(wu * nbody9_u));
        std::ranges::copy(proto.xanchor, xanchor_batch.begin() + static_cast<std::ptrdiff_t>(wu * njnt3_u));
        std::ranges::copy(proto.xaxis, xaxis_batch.begin() + static_cast<std::ptrdiff_t>(wu * njnt3_u));
        std::ranges::copy(proto.tendon_range, tr_batch.begin() + static_cast<std::ptrdiff_t>(wu * tr_u));
        std::ranges::copy(proto.dof_frictionloss, fl_batch.begin() + static_cast<std::ptrdiff_t>(wu * nv_u));
        std::ranges::copy(proto.sensordata, sd_batch.begin() + static_cast<std::ptrdiff_t>(wu * nsd_u));
        if (d0->nefc > 0) {
            std::ranges::copy(proto.efc_type, et_batch.begin() + static_cast<std::ptrdiff_t>(wu * njmax_u));
            std::ranges::copy(proto.efc_id, eid_batch.begin() + static_cast<std::ptrdiff_t>(wu * njmax_u));
            std::ranges::copy(proto.efc_J, eJ_batch.begin() + static_cast<std::ptrdiff_t>(wu * eJ_world_u));
            std::ranges::copy(proto.efc_force, ef_batch.begin() + static_cast<std::ptrdiff_t>(wu * njmax_u));
        }
        for (int c = 0; c < d0->ncon; ++c) {
            const std::size_t gi = wu * to_index(d0->ncon) + to_index(c);
            c_wid_batch[gi] = w;
            c_geom_batch[2ULL * gi + 0ULL] = proto.contact_geom[to_index(c) * 2ULL + 0ULL];
            c_geom_batch[2ULL * gi + 1ULL] = proto.contact_geom[to_index(c) * 2ULL + 1ULL];
            c_dim_batch[gi] = proto.contact_dim[to_index(c)];
            for (int k = 0; k < 4; ++k) {
                c_adr_batch[4ULL * gi + to_index(k)] = proto.contact_efc_address[to_index(c) * 4ULL + to_index(k)];
            }
        }
    }

    // Run host batch (nworld=64)
    std::vector<bike::gpu::GpuSp3WorldState> host_sp3(kNwU);
    std::vector<bike::gpu::GpuWorldState> host_ws(kNwU);
    std::vector<float> host_qpos = qpos_batch;
    std::vector<float> host_qfrc = qfrc_batch;
    std::vector<float> host_ctrl = ctrl_batch;
    std::vector<float> host_tr = tr_batch;
    std::vector<float> host_fl = fl_batch;

    require(bike_gpu_launch_sp3_init(
                nullptr, kNworld, &idx, cfgs.data(), 1, host_sp3.data(), host_ws.data(),
                host_qpos.data(), 0.0f, ntendon, host_tr.data(), tr_stride) == 0,
            "host sp3 batch init failed");
    require(bike_gpu_launch_sp3_pre_solve(
                nullptr, kNworld, kDtS, &idx, &tt->tyre, &tt->terrain_desc,
                tt->terrain_f32.data(), tt->terrain_i32.data(),
                cfgs.data(), 1, host_sp3.data(), host_ws.data(),
                host_qpos.data(), qvel_batch.data(), xpos_batch.data(), xmat_batch.data(),
                xanchor_batch.data(), xaxis_batch.data(), host_qfrc.data(), host_ctrl.data(),
                ntendon, host_tr.data(), tr_stride, host_fl.data(), nv) == 0,
            "host sp3 batch pre_solve failed");
    require(bike_gpu_launch_sp3_post_solve(
                nullptr, kNworld, kDtS, &idx, cfgs.data(), 1, host_sp3.data(),
                host_qpos.data(), qvel_batch.data(), xmat_batch.data(),
                nsd, sd_batch.data(), nefc_batch.data(), njmax, nv,
                et_batch.data(), eid_batch.data(), eJ_batch.data(), ef_batch.data(),
                nacon_total.data(), nconmax, 4,
                c_wid_batch.data(), c_geom_batch.data(), c_adr_batch.data(), c_dim_batch.data()) == 0,
            "host sp3 batch post_solve failed");

    // Host 1-step multi-world isolation on selected worlds {0, 17, 63}
    for (const int w : {0, 17, 63}) {
        const std::size_t wu = to_index(w);
        const auto solo_sp3 = std::make_unique<bike::gpu::GpuSp3WorldState>();
        const auto solo_ws = std::make_unique<bike::gpu::GpuWorldState>();
        std::vector<float> solo_qpos(qpos_batch.begin() + static_cast<std::ptrdiff_t>(wu * nq_u),
                                     qpos_batch.begin() + static_cast<std::ptrdiff_t>((wu + 1ULL) * nq_u));
        std::vector<float> solo_qfrc(nv_u, 0.0f);
        std::vector<float> solo_ctrl(to_index(nu), 0.0f);
        std::vector<float> solo_tr(tr_batch.begin() + static_cast<std::ptrdiff_t>(wu * tr_u),
                                   tr_batch.begin() + static_cast<std::ptrdiff_t>((wu + 1ULL) * tr_u));
        std::vector<float> solo_fl(fl_batch.begin() + static_cast<std::ptrdiff_t>(wu * nv_u),
                                   fl_batch.begin() + static_cast<std::ptrdiff_t>((wu + 1ULL) * nv_u));

        require(bike_gpu_launch_sp3_init(
                    nullptr, 1, &idx, &cfgs[wu], 0, solo_sp3.get(), solo_ws.get(),
                    solo_qpos.data(), 0.0f, ntendon, solo_tr.data(), tr_stride) == 0,
                "host solo init failed");
        require(bike_gpu_launch_sp3_pre_solve(
                    nullptr, 1, kDtS, &idx, &tt->tyre, &tt->terrain_desc,
                    tt->terrain_f32.data(), tt->terrain_i32.data(),
                    &cfgs[wu], 0, solo_sp3.get(), solo_ws.get(),
                    solo_qpos.data(),
                    std::span{qvel_batch}.subspan(wu * nv_u).data(),
                    std::span{xpos_batch}.subspan(wu * nbody3_u).data(),
                    std::span{xmat_batch}.subspan(wu * nbody9_u).data(),
                    std::span{xanchor_batch}.subspan(wu * njnt3_u).data(),
                    std::span{xaxis_batch}.subspan(wu * njnt3_u).data(),
                    solo_qfrc.data(), solo_ctrl.data(),
                    ntendon, solo_tr.data(), tr_stride, solo_fl.data(), nv) == 0,
                "host solo pre_solve failed");

        std::vector<int> solo_c_geom;
        std::vector<int> solo_c_adr;
        std::vector<int> solo_c_dim;
        std::vector<int> solo_c_wid;
        for (int c = 0; c < d0->ncon; ++c) {
            solo_c_wid.push_back(0);
            solo_c_geom.push_back(proto.contact_geom[to_index(c) * 2ULL + 0ULL]);
            solo_c_geom.push_back(proto.contact_geom[to_index(c) * 2ULL + 1ULL]);
            solo_c_dim.push_back(proto.contact_dim[to_index(c)]);
            for (int k = 0; k < 4; ++k) {
                solo_c_adr.push_back(proto.contact_efc_address[to_index(c) * 4ULL + to_index(k)]);
            }
        }
        const int solo_nacon = d0->ncon;
        require(bike_gpu_launch_sp3_post_solve(
                    nullptr, 1, kDtS, &idx, &cfgs[wu], 0, solo_sp3.get(),
                    solo_qpos.data(),
                    std::span{qvel_batch}.subspan(wu * nv_u).data(),
                    std::span{xmat_batch}.subspan(wu * nbody9_u).data(),
                    nsd, std::span{sd_batch}.subspan(wu * nsd_u).data(),
                    &nefc_batch[wu], njmax, nv,
                    std::span{et_batch}.subspan(wu * njmax_u).data(),
                    std::span{eid_batch}.subspan(wu * njmax_u).data(),
                    std::span{eJ_batch}.subspan(wu * eJ_world_u).data(),
                    std::span{ef_batch}.subspan(wu * njmax_u).data(),
                    &solo_nacon, std::max(1, solo_nacon), 4,
                    solo_c_wid.data(), solo_c_geom.data(), solo_c_adr.data(), solo_c_dim.data()) == 0,
                "host solo post_solve failed");

        for (int i = 0; i < nv; ++i) {
            require(solo_qfrc[to_index(i)] == host_qfrc[wu * to_index(nv) + to_index(i)],
                    "host solo vs batch qfrc isolation diff != 0");
            require(solo_fl[to_index(i)] == host_fl[wu * to_index(nv) + to_index(i)],
                    "host solo vs batch frictionloss isolation diff != 0");
        }
        for (int i = 0; i < tr_stride; ++i) {
            require(solo_tr[to_index(i)] == host_tr[wu * to_index(tr_stride) + to_index(i)],
                    "host solo vs batch tendon_range isolation diff != 0");
        }
        require(solo_sp3->last_efc.delivered_crank_torque_nm ==
                    host_sp3[wu].last_efc.delivered_crank_torque_nm &&
                solo_sp3->last_efc.hand_press_n == host_sp3[wu].last_efc.hand_press_n &&
                solo_sp3->step_index == host_sp3[wu].step_index,
                "host solo vs batch sp3_state isolation diff != 0");
    }

#if defined(__APPLE__)
    const std::string gpu_dir = (repo_root() / "native/src/gpu").string();
    require(bike_metal_init(gpu_dir.c_str()) == 0, "bike_metal_init failed");
    require(bike_metal_sizeof_sp3_config() == bike_gpu_sizeof_sp3_config(),
            "Metal vs Host sizeof(GpuSp3Config) mismatch");
    require(bike_metal_sizeof_sp3_world_state() == bike_gpu_sizeof_sp3_world_state(),
            "Metal vs Host sizeof(GpuSp3WorldState) mismatch");
    require(bike_metal_sizeof_sp3_world_state() <= 16384,
            "GpuSp3WorldState exceeds 16 KiB/world");

    std::vector<bike::gpu::GpuSp3WorldState> mtl_sp3(to_index(kNworld));
    std::vector<bike::gpu::GpuWorldState> mtl_ws(to_index(kNworld));
    std::vector<float> mtl_qpos = qpos_batch;
    std::vector<float> mtl_qfrc = qfrc_batch;
    std::vector<float> mtl_ctrl = ctrl_batch;
    std::vector<float> mtl_tr = tr_batch;
    std::vector<float> mtl_fl = fl_batch;

    require(bike_metal_launch_sp3_init(
                kNworld, &idx, cfgs.data(), 1, mtl_sp3.data(), mtl_ws.data(),
                mtl_qpos.data(), 0.0f, ntendon, mtl_tr.data(), tr_stride) == 0,
            "Metal sp3 init failed");
    require(bike_metal_launch_sp3_pre_solve(
                kNworld, kDtS, &idx, &tt->tyre, &tt->terrain_desc,
                tt->terrain_f32.data(), tt->terrain_i32.data(),
                cfgs.data(), 1, mtl_sp3.data(), mtl_ws.data(),
                mtl_qpos.data(), qvel_batch.data(), xpos_batch.data(), xmat_batch.data(),
                xanchor_batch.data(), xaxis_batch.data(), mtl_qfrc.data(), mtl_ctrl.data(),
                ntendon, mtl_tr.data(), tr_stride, mtl_fl.data(), nv) == 0,
            "Metal sp3 pre_solve failed");
    require(bike_metal_launch_sp3_post_solve(
                kNworld, kDtS, &idx, cfgs.data(), 1, mtl_sp3.data(),
                mtl_qpos.data(), qvel_batch.data(), xmat_batch.data(),
                nsd, sd_batch.data(), nefc_batch.data(), njmax, nv,
                et_batch.data(), eid_batch.data(), eJ_batch.data(), ef_batch.data(),
                nacon_total.data(), nconmax, 4,
                c_wid_batch.data(), c_geom_batch.data(), c_adr_batch.data(), c_dim_batch.data()) == 0,
            "Metal sp3 post_solve failed");

    // Check Host vs Metal parity across all 64 worlds
    double max_dqfrc = 0.0;
    double max_dctrl = 0.0;
    double max_dtr = 0.0;
    double max_dfl = 0.0;
    double max_deq_crank = 0.0;
    double max_dhand = 0.0;
    for (int w = 0; w < kNworld; ++w) {
        const std::size_t wu = to_index(w);
        for (int i = 0; i < nv; ++i) {
            max_dqfrc = std::max(max_dqfrc, std::abs(static_cast<double>(
                host_qfrc[wu * to_index(nv) + to_index(i)] - mtl_qfrc[wu * to_index(nv) + to_index(i)])));
            max_dfl = std::max(max_dfl, std::abs(static_cast<double>(
                host_fl[wu * to_index(nv) + to_index(i)] - mtl_fl[wu * to_index(nv) + to_index(i)])));
        }
        for (int i = 0; i < nu; ++i) {
            max_dctrl = std::max(max_dctrl, std::abs(static_cast<double>(
                host_ctrl[wu * to_index(nu) + to_index(i)] - mtl_ctrl[wu * to_index(nu) + to_index(i)])));
        }
        for (int i = 0; i < tr_stride; ++i) {
            max_dtr = std::max(max_dtr, std::abs(static_cast<double>(
                host_tr[wu * to_index(tr_stride) + to_index(i)] - mtl_tr[wu * to_index(tr_stride) + to_index(i)])));
        }
        max_deq_crank = std::max(max_deq_crank, std::abs(static_cast<double>(
            host_sp3[wu].last_efc.delivered_crank_torque_nm - mtl_sp3[wu].last_efc.delivered_crank_torque_nm)));
        max_dhand = std::max(max_dhand, std::abs(static_cast<double>(
            host_sp3[wu].last_efc.hand_press_n - mtl_sp3[wu].last_efc.hand_press_n)));
    }
    std::cout << "[test_host_vs_metal] device=" << bike_metal_device_name()
              << " sizeof(GpuSp3Config)=" << bike_metal_sizeof_sp3_config()
              << " sizeof(GpuSp3WorldState)=" << bike_metal_sizeof_sp3_world_state()
              << " max_dqfrc=" << max_dqfrc << " max_dctrl=" << max_dctrl
              << " max_dtr=" << max_dtr << " max_dfl=" << max_dfl
              << " max_dcrank=" << max_deq_crank << " max_dhand=" << max_dhand << '\n';
    require(max_dqfrc <= 1e-5, "Host vs Metal qfrc parity exceeded 1e-5");
    require(max_dctrl <= 1e-5, "Host vs Metal ctrl parity exceeded 1e-5");
    require(max_dtr <= 1e-5, "Host vs Metal tendon_range parity exceeded 1e-5");
    require(max_dfl <= 1e-5, "Host vs Metal dof_frictionloss parity exceeded 1e-5");
    require(max_deq_crank <= 1e-5, "Host vs Metal delivered_crank_torque_nm parity exceeded 1e-5");
    require(max_dhand <= 1e-5, "Host vs Metal hand_press_n parity exceeded 1e-5");

    // Metal 1-step solo (nworld=1) vs batch (nworld=64) isolation must be 0.0 bit diff
    {
        const int w = 17;
        const std::size_t wu = to_index(w);
        const auto msolo_sp3 = std::make_unique<bike::gpu::GpuSp3WorldState>();
        const auto msolo_ws = std::make_unique<bike::gpu::GpuWorldState>();
        std::vector<float> msolo_qpos(qpos_batch.begin() + static_cast<std::ptrdiff_t>(wu * nq_u),
                                      qpos_batch.begin() + static_cast<std::ptrdiff_t>((wu + 1ULL) * nq_u));
        std::vector<float> msolo_qfrc(nv_u, 0.0f);
        std::vector<float> msolo_ctrl(to_index(nu), 0.0f);
        std::vector<float> msolo_tr(tr_batch.begin() + static_cast<std::ptrdiff_t>(wu * tr_u),
                                    tr_batch.begin() + static_cast<std::ptrdiff_t>((wu + 1ULL) * tr_u));
        std::vector<float> msolo_fl(fl_batch.begin() + static_cast<std::ptrdiff_t>(wu * nv_u),
                                    fl_batch.begin() + static_cast<std::ptrdiff_t>((wu + 1ULL) * nv_u));

        require(bike_metal_launch_sp3_init(
                    1, &idx, &cfgs[wu], 0, msolo_sp3.get(), msolo_ws.get(),
                    msolo_qpos.data(), 0.0f, ntendon, msolo_tr.data(), tr_stride) == 0,
                "Metal solo init failed");
        require(bike_metal_launch_sp3_pre_solve(
                    1, kDtS, &idx, &tt->tyre, &tt->terrain_desc,
                    tt->terrain_f32.data(), tt->terrain_i32.data(),
                    &cfgs[wu], 0, msolo_sp3.get(), msolo_ws.get(),
                    msolo_qpos.data(),
                    std::span{qvel_batch}.subspan(wu * nv_u).data(),
                    std::span{xpos_batch}.subspan(wu * nbody3_u).data(),
                    std::span{xmat_batch}.subspan(wu * nbody9_u).data(),
                    std::span{xanchor_batch}.subspan(wu * njnt3_u).data(),
                    std::span{xaxis_batch}.subspan(wu * njnt3_u).data(),
                    msolo_qfrc.data(), msolo_ctrl.data(),
                    ntendon, msolo_tr.data(), tr_stride, msolo_fl.data(), nv) == 0,
                "Metal solo pre_solve failed");

        std::vector<int> sc_wid;
        std::vector<int> sc_geom;
        std::vector<int> sc_adr;
        std::vector<int> sc_dim;
        for (int c = 0; c < d0->ncon; ++c) {
            sc_wid.push_back(0);
            sc_geom.push_back(proto.contact_geom[to_index(c) * 2ULL + 0ULL]);
            sc_geom.push_back(proto.contact_geom[to_index(c) * 2ULL + 1ULL]);
            sc_dim.push_back(proto.contact_dim[to_index(c)]);
            for (int k = 0; k < 4; ++k) {
                sc_adr.push_back(proto.contact_efc_address[to_index(c) * 4ULL + to_index(k)]);
            }
        }
        const int solo_nacon = d0->ncon;
        require(bike_metal_launch_sp3_post_solve(
                    1, kDtS, &idx, &cfgs[wu], 0, msolo_sp3.get(),
                    msolo_qpos.data(),
                    std::span{qvel_batch}.subspan(wu * nv_u).data(),
                    std::span{xmat_batch}.subspan(wu * nbody9_u).data(),
                    nsd, std::span{sd_batch}.subspan(wu * nsd_u).data(),
                    &nefc_batch[wu], njmax, nv,
                    std::span{et_batch}.subspan(wu * njmax_u).data(),
                    std::span{eid_batch}.subspan(wu * njmax_u).data(),
                    std::span{eJ_batch}.subspan(wu * eJ_world_u).data(),
                    std::span{ef_batch}.subspan(wu * njmax_u).data(),
                    &solo_nacon, std::max(1, solo_nacon), 4,
                    sc_wid.data(), sc_geom.data(), sc_adr.data(), sc_dim.data()) == 0,
                "Metal solo post_solve failed");

        for (int i = 0; i < nv; ++i) {
            require(msolo_qfrc[to_index(i)] == mtl_qfrc[wu * to_index(nv) + to_index(i)],
                    "Metal solo vs batch qfrc isolation diff != 0");
            require(msolo_fl[to_index(i)] == mtl_fl[wu * to_index(nv) + to_index(i)],
                    "Metal solo vs batch frictionloss isolation diff != 0");
        }
        for (int i = 0; i < tr_stride; ++i) {
            require(msolo_tr[to_index(i)] == mtl_tr[wu * to_index(tr_stride) + to_index(i)],
                    "Metal solo vs batch tendon_range isolation diff != 0");
        }
        require(msolo_sp3->last_efc.delivered_crank_torque_nm ==
                    mtl_sp3[wu].last_efc.delivered_crank_torque_nm &&
                msolo_sp3->last_efc.hand_press_n == mtl_sp3[wu].last_efc.hand_press_n,
                "Metal solo vs batch sp3_state isolation diff != 0");
    }
#endif
}

void test_sp3_assist_coast_gate_and_launch_preload(const TempSp3Models &models) {
    ModelPtr m13 = load_xml(models.xml13);
    const auto idx13 = resolve_model_indices(m13.get());
    const auto tt13 = build_tyre_terrain_from_xml(models.xml13, idx13);
    const int nv13 = to_int(m13->nv);
    const int nt13 = to_int(m13->ntendon);
    const int nsd13 = to_int(m13->nsensordata);

    // (a) Active pedaling in apply_direct_crank_torque=1 must produce last_motor_torque_nm > 0.
    const auto cfg = build_sp3_config_for_model(m13.get());
    cfg->apply_direct_crank_torque = 1;
    cfg->intent.enabled = 0;
    cfg->shifting.enabled = 0;
    cfg->caller_human_torque_nm = 25.0f;
    cfg->caller_motor_torque_nm = -1.0f;
    cfg->caller_motor_limit_nm = -1.0f;

    DataPtr d_ped(nullptr, &mj_deleteData);
    build_riding_initial_state(m13.get(), idx13, {.speed_mps = 2.5, .crank_omega_rad_s = 7.5}, d_ped);
    auto b_ped = snapshot_mj_to_float(m13.get(), d_ped.get());
    const auto st_ped = std::make_unique<bike::gpu::GpuSp3WorldState>();
    const auto ws_ped = std::make_unique<bike::gpu::GpuWorldState>();
    require(bike_gpu_launch_sp3_init(
                nullptr, 1, &idx13, cfg.get(), 0, st_ped.get(), ws_ped.get(),
                b_ped.qpos.data(), 0.0f, nt13, b_ped.tendon_range.data(), 2 * nt13) == 0,
            "b1 assist init failed");

    for (int step = 0; step < 30; ++step) {
        b_ped = snapshot_mj_to_float(m13.get(), d_ped.get());
        require(bike_gpu_launch_sp3_pre_solve(
                    nullptr, 1, kDtS, &idx13, &tt13->tyre, &tt13->terrain_desc,
                    tt13->terrain_f32.data(), tt13->terrain_i32.data(),
                    cfg.get(), 0, st_ped.get(), ws_ped.get(),
                    b_ped.qpos.data(), b_ped.qvel.data(), b_ped.xpos.data(), b_ped.xmat.data(),
                    b_ped.xanchor.data(), b_ped.xaxis.data(), b_ped.qfrc.data(), b_ped.ctrl.data(),
                    nt13, b_ped.tendon_range.data(), 2 * nt13,
                    b_ped.dof_frictionloss.data(), nv13) == 0,
                "b1 pedaling pre_solve failed");
        write_pre_solve_back_to_mj(m13.get(), d_ped.get(), b_ped);
        mj_step(m13.get(), d_ped.get());
        b_ped = snapshot_mj_to_float(m13.get(), d_ped.get());
        require(bike_gpu_launch_sp3_post_solve(
                    nullptr, 1, kDtS, &idx13, cfg.get(), 0, st_ped.get(),
                    b_ped.qpos.data(), b_ped.qvel.data(), b_ped.xmat.data(),
                    nsd13, b_ped.sensordata.data(),
                    b_ped.nefc.data(), std::max(1, d_ped->nefc), nv13,
                    b_ped.efc_type.data(), b_ped.efc_id.data(), b_ped.efc_J.data(), b_ped.efc_force.data(),
                    b_ped.nacon.data(), std::max(1, d_ped->ncon), 4,
                    b_ped.contact_worldid.data(), b_ped.contact_geom.data(),
                    b_ped.contact_efc_address.data(), b_ped.contact_dim.data()) == 0,
                "b1 pedaling post_solve failed");
    }
    if (!(st_ped->last_motor_torque_nm > 0.0f)) {
        std::cerr << "B1(a) diag: motor=" << st_ped->last_motor_torque_nm
                  << " cmd_crank_tq=" << st_ped->last_commanded_crank_torque_nm
                  << " ped_effort=" << st_ped->pedaling.effort
                  << " ped_coast=" << static_cast<int>(st_ped->pedaling.coasting)
                  << " assist_gain=" << st_ped->assist.last_gain
                  << " assist_tq=" << st_ped->assist.torque
                  << " ds_w=" << b_ped.qvel[to_index(idx13.drive_shaft_spin_dof)]
                  << " rw_w=" << b_ped.qvel[to_index(idx13.rear_wheel_spin_dof)]
                  << " crank_w=" << b_ped.qvel[to_index(idx13.crank_spin_dof)]
                  << " speed=" << b_ped.qvel[to_index(idx13.root_x_dof)]
                  << " fh_tid=" << cfg->efc_scan.freehub_tendon_id
                  << " active_cog=" << st_ped->shifting.active_cog_idx
                  << '\n';
    }
    require(st_ped->last_motor_torque_nm > 0.0f,
            "B1(a): active pedaling with apply_direct_crank_torque=1 must produce last_motor_torque_nm > 0");

    // (b) During PedalMode::coasting (cadence overspeed, no braking), assist receives 0.0f
    // even when last_efc.delivered_crank_torque_nm > 10.0f.
    const float kTwoPi = 6.28318530717958647692f;
    const float overspeed_crank_rad_s = 130.0f * kTwoPi / 60.0f;
    auto b_coast = snapshot_mj_to_float(m13.get(), d_ped.get());
    b_coast.qvel[to_index(idx13.crank_spin_dof)] = overspeed_crank_rad_s;

    const auto cfg_coast_zero = std::make_unique<bike::gpu::GpuSp3Config>(*cfg);
    cfg_coast_zero->apply_direct_crank_torque = 0;
    cfg_coast_zero->pedaling.coast_cadence_tau_s = 0.0f;
    const auto st_coast_zero = std::make_unique<bike::gpu::GpuSp3WorldState>(*st_ped);
    const auto ws_coast_zero = std::make_unique<bike::gpu::GpuWorldState>(*ws_ped);
    st_coast_zero->last_efc.delivered_crank_torque_nm = 0.0f;

    const auto st_coast_loaded = std::make_unique<bike::gpu::GpuSp3WorldState>(*st_ped);
    const auto ws_coast_loaded = std::make_unique<bike::gpu::GpuWorldState>(*ws_ped);
    st_coast_loaded->last_efc.delivered_crank_torque_nm = 25.0f;
    auto b_coast_loaded = b_coast;

    require(bike_gpu_launch_sp3_pre_solve(
                nullptr, 1, kDtS, &idx13, &tt13->tyre, &tt13->terrain_desc,
                tt13->terrain_f32.data(), tt13->terrain_i32.data(),
                cfg_coast_zero.get(), 0, st_coast_zero.get(), ws_coast_zero.get(),
                b_coast.qpos.data(), b_coast.qvel.data(), b_coast.xpos.data(), b_coast.xmat.data(),
                b_coast.xanchor.data(), b_coast.xaxis.data(), b_coast.qfrc.data(), b_coast.ctrl.data(),
                nt13, b_coast.tendon_range.data(), 2 * nt13,
                b_coast.dof_frictionloss.data(), nv13) == 0,
            "b1 coast zero pre_solve failed");
    require(bike_gpu_launch_sp3_pre_solve(
                nullptr, 1, kDtS, &idx13, &tt13->tyre, &tt13->terrain_desc,
                tt13->terrain_f32.data(), tt13->terrain_i32.data(),
                cfg_coast_zero.get(), 0, st_coast_loaded.get(), ws_coast_loaded.get(),
                b_coast_loaded.qpos.data(), b_coast_loaded.qvel.data(), b_coast_loaded.xpos.data(),
                b_coast_loaded.xmat.data(), b_coast_loaded.xanchor.data(), b_coast_loaded.xaxis.data(),
                b_coast_loaded.qfrc.data(), b_coast_loaded.ctrl.data(),
                nt13, b_coast_loaded.tendon_range.data(), 2 * nt13,
                b_coast_loaded.dof_frictionloss.data(), nv13) == 0,
            "b1 coast loaded pre_solve failed");
    require(st_coast_loaded->pedaling.coasting != 0,
            "B1(b): overspeed crank must enter PedalMode::coasting");
    require(st_coast_loaded->last_motor_torque_nm == st_coast_zero->last_motor_torque_nm,
            "B1(b): assist must receive 0.0f sensor torque during coasting despite delivered_crank_torque_nm > 10");

    // (c) Standstill hill-launch (arm_standstill_start=1, grade=0.10f) in apply_direct_crank_torque=1
    // must transition out of LaunchPhase::hold without preload_missed.
    const auto cfg_launch = build_sp3_config_for_model(m13.get());
    cfg_launch->apply_direct_crank_torque = 1;
    cfg_launch->seed_lean_from_launch = 1;
    cfg_launch->arm_standstill_start = 1;
    cfg_launch->caller_human_torque_nm = -1.0f;
    cfg_launch->caller_motor_torque_nm = -1.0f;
    cfg_launch->caller_motor_limit_nm = -1.0f;
    for (float &rel_tq : std::span{cfg_launch->launch.release_torque_nm}) {
        rel_tq = 9.0f;
    }

    DataPtr d_launch(nullptr, &mj_deleteData);
    build_riding_initial_state(m13.get(), idx13, {.speed_mps = 0.0, .crank_omega_rad_s = 0.0}, d_launch);
    auto b_launch = snapshot_mj_to_float(m13.get(), d_launch.get());
    const auto st_launch = std::make_unique<bike::gpu::GpuSp3WorldState>();
    const auto ws_launch = std::make_unique<bike::gpu::GpuWorldState>();
    require(bike_gpu_launch_sp3_init(
                nullptr, 1, &idx13, cfg_launch.get(), 0, st_launch.get(), ws_launch.get(),
                b_launch.qpos.data(), 0.10f, nt13, b_launch.tendon_range.data(), 2 * nt13) == 0,
            "b1 launch init failed");
    require(st_launch->sp3.intent.launch.phase == bike::physics_core::LaunchPhase::hold,
            "B1(c): standstill launch init must start in LaunchPhase::hold");

    bool left_hold = false;
    for (int step = 0; step < 600; ++step) {
        b_launch = snapshot_mj_to_float(m13.get(), d_launch.get());
        require(bike_gpu_launch_sp3_pre_solve(
                    nullptr, 1, kDtS, &idx13, &tt13->tyre, &tt13->terrain_desc,
                    tt13->terrain_f32.data(), tt13->terrain_i32.data(),
                    cfg_launch.get(), 0, st_launch.get(), ws_launch.get(),
                    b_launch.qpos.data(), b_launch.qvel.data(), b_launch.xpos.data(),
                    b_launch.xmat.data(), b_launch.xanchor.data(), b_launch.xaxis.data(),
                    b_launch.qfrc.data(), b_launch.ctrl.data(),
                    nt13, b_launch.tendon_range.data(), 2 * nt13,
                    b_launch.dof_frictionloss.data(), nv13) == 0,
                "b1 launch pre_solve failed");
        write_pre_solve_back_to_mj(m13.get(), d_launch.get(), b_launch);
        mj_step(m13.get(), d_launch.get());
        b_launch = snapshot_mj_to_float(m13.get(), d_launch.get());
        require(bike_gpu_launch_sp3_post_solve(
                    nullptr, 1, kDtS, &idx13, cfg_launch.get(), 0, st_launch.get(),
                    b_launch.qpos.data(), b_launch.qvel.data(), b_launch.xmat.data(),
                    nsd13, b_launch.sensordata.data(),
                    b_launch.nefc.data(), std::max(1, d_launch->nefc), nv13,
                    b_launch.efc_type.data(), b_launch.efc_id.data(), b_launch.efc_J.data(),
                    b_launch.efc_force.data(),
                    b_launch.nacon.data(), std::max(1, d_launch->ncon), 4,
                    b_launch.contact_worldid.data(), b_launch.contact_geom.data(),
                    b_launch.contact_efc_address.data(), b_launch.contact_dim.data()) == 0,
                "b1 launch post_solve failed");
        if (st_launch->sp3.intent.launch.phase != bike::physics_core::LaunchPhase::hold) {
            left_hold = true;
            break;
        }
    }
    require(left_hold,
            "B1(c): standstill launch must transition out of LaunchPhase::hold in apply_direct_crank_torque=1 mode");
    require(st_launch->sp3.intent.launch.preload_missed == 0,
            "B1(c): standstill launch must release hold via preload without preload_missed");
    std::cout << "[test_sp3_assist_coast_gate_and_launch_preload] motor_ped="
              << st_ped->last_motor_torque_nm << " motor_coast=" << st_coast_loaded->last_motor_torque_nm
              << " launch_phase=" << static_cast<int>(st_launch->sp3.intent.launch.phase) << '\n';
}

[[gnu::noinline]] void test_sp3_cruise_and_ride_program(const TempSp3Models &models) {
    ModelPtr m13 = load_xml(models.xml13);
    const auto idx13 = resolve_model_indices(m13.get());
    const auto tt13 = build_tyre_terrain_from_xml(models.xml13, idx13);
    const int nv13 = to_int(m13->nv);
    const int nt13 = to_int(m13->ntendon);
    const int nsd13 = to_int(m13->nsensordata);

    // 1. Cruise controller (cruise_enabled = 1): engages when grounded & below target speed, suppressed when braking.
    {
        const auto cfg = build_sp3_config_for_model(m13.get());
        cfg->intent.enabled = 0;
        cfg->shifting.enabled = 0;
        cfg->cruise_enabled = 1;
        cfg->cruise.target_speed_kmh = 25.0f;
        cfg->cruise.kp_nm_per_mps = 40.0f;
        cfg->cruise.ki_nm_per_mps_s = 10.0f;
        cfg->cruise.torque_ceiling_nm = 120.0f;

        DataPtr d(nullptr, &mj_deleteData);
        build_riding_initial_state(m13.get(), idx13, {.speed_mps = 4.5, .crank_omega_rad_s = 6.0}, d);
        auto b = snapshot_mj_to_float(m13.get(), d.get());
        const auto st = std::make_unique<bike::gpu::GpuSp3WorldState>();
        const auto ws = std::make_unique<bike::gpu::GpuWorldState>();
        require(bike_gpu_launch_sp3_init(
                    nullptr, 1, &idx13, cfg.get(), 0, st.get(), ws.get(),
                    b.qpos.data(), 0.0f, nt13, b.tendon_range.data(), 2 * nt13) == 0,
                "cruise init failed");

        for (int step = 0; step < 6; ++step) {
            b = snapshot_mj_to_float(m13.get(), d.get());
            require(bike_gpu_launch_sp3_pre_solve(
                        nullptr, 1, kDtS, &idx13, &tt13->tyre, &tt13->terrain_desc,
                        tt13->terrain_f32.data(), tt13->terrain_i32.data(),
                        cfg.get(), 0, st.get(), ws.get(),
                        b.qpos.data(), b.qvel.data(), b.xpos.data(), b.xmat.data(),
                        b.xanchor.data(), b.xaxis.data(), b.qfrc.data(), b.ctrl.data(),
                        nt13, b.tendon_range.data(), 2 * nt13,
                        b.dof_frictionloss.data(), nv13) == 0,
                    "cruise pre_solve failed");
            write_pre_solve_back_to_mj(m13.get(), d.get(), b);
            mj_step(m13.get(), d.get());
            b = snapshot_mj_to_float(m13.get(), d.get());
            require(bike_gpu_launch_sp3_post_solve(
                        nullptr, 1, kDtS, &idx13, cfg.get(), 0, st.get(),
                        b.qpos.data(), b.qvel.data(), b.xmat.data(),
                        nsd13, b.sensordata.data(),
                        b.nefc.data(), std::max(1, d->nefc), nv13,
                        b.efc_type.data(), b.efc_id.data(), b.efc_J.data(), b.efc_force.data(),
                        b.nacon.data(), std::max(1, d->ncon), 4,
                        b.contact_worldid.data(), b.contact_geom.data(),
                        b.contact_efc_address.data(), b.contact_dim.data()) == 0,
                    "cruise post_solve failed");
        }
        require(st->cruise.engaged != 0 && st->last_cruise_torque_nm > 1.0f,
                "Cruise (cruise_enabled=1) must engage and apply positive rear wheel torque below target speed");

        cfg->caller_braking = 1;
        b = snapshot_mj_to_float(m13.get(), d.get());
        require(bike_gpu_launch_sp3_pre_solve(
                    nullptr, 1, kDtS, &idx13, &tt13->tyre, &tt13->terrain_desc,
                    tt13->terrain_f32.data(), tt13->terrain_i32.data(),
                    cfg.get(), 0, st.get(), ws.get(),
                    b.qpos.data(), b.qvel.data(), b.xpos.data(), b.xmat.data(),
                    b.xanchor.data(), b.xaxis.data(), b.qfrc.data(), b.ctrl.data(),
                    nt13, b.tendon_range.data(), 2 * nt13,
                    b.dof_frictionloss.data(), nv13) == 0,
                "cruise braking pre_solve failed");
        require(st->last_cruise_torque_nm == 0.0f,
                "Cruise torque must zero when braking");
    }

    // 2. RideProgram (program_enabled = 1): step segment + linear ramp segment.
    {
        const auto cfg = build_sp3_config_for_model(m13.get());
        cfg->intent.enabled = 0;
        cfg->shifting.enabled = 0;
        cfg->pedaling.effort_slew_nm_s = 100000.0f;
        cfg->program_enabled = 1;
        cfg->ride_program.segment_count = 3;
        cfg->ride_program.reaction_delay_s = 0.0f;
        const std::span segs(cfg->ride_program.segments);
        segs[0].t_end_s = 0.01f;
        segs[0].channel_mask =
            static_cast<bike::physics_core::u32>(bike::physics_core::GpuProgramChannelMask::has_human_torque);
        segs[0].human_torque_nm = 32.0f;

        segs[1].t_end_s = 0.02f;
        segs[1].channel_mask =
            static_cast<bike::physics_core::u32>(bike::physics_core::GpuProgramChannelMask::has_human_torque) |
            static_cast<bike::physics_core::u32>(bike::physics_core::GpuProgramChannelMask::linear_ramp);
        segs[1].human_torque_nm = 52.0f;

        segs[2].t_end_s = 0.03f;
        segs[2].channel_mask =
            static_cast<bike::physics_core::u32>(bike::physics_core::GpuProgramChannelMask::has_human_torque) |
            static_cast<bike::physics_core::u32>(bike::physics_core::GpuProgramChannelMask::has_front_brake);
        segs[2].human_torque_nm = 32.0f;
        segs[2].front_brake = 0.25f;

        DataPtr d(nullptr, &mj_deleteData);
        build_riding_initial_state(m13.get(), idx13, {.speed_mps = 3.0, .crank_omega_rad_s = 6.0}, d);
        auto b = snapshot_mj_to_float(m13.get(), d.get());
        const auto st = std::make_unique<bike::gpu::GpuSp3WorldState>();
        const auto ws = std::make_unique<bike::gpu::GpuWorldState>();
        require(bike_gpu_launch_sp3_init(
                    nullptr, 1, &idx13, cfg.get(), 0, st.get(), ws.get(),
                    b.qpos.data(), 0.0f, nt13, b.tendon_range.data(), 2 * nt13) == 0,
                "ride_program init failed");

        // Segment 0 step override at t = 0.0 s (step_index = 0).
        st->step_index = 0;
        require(bike_gpu_launch_sp3_pre_solve(
                    nullptr, 1, kDtS, &idx13, &tt13->tyre, &tt13->terrain_desc,
                    tt13->terrain_f32.data(), tt13->terrain_i32.data(),
                    cfg.get(), 0, st.get(), ws.get(),
                    b.qpos.data(), b.qvel.data(), b.xpos.data(), b.xmat.data(),
                    b.xanchor.data(), b.xaxis.data(), b.qfrc.data(), b.ctrl.data(),
                    nt13, b.tendon_range.data(), 2 * nt13,
                    b.dof_frictionloss.data(), nv13) == 0,
                "ride_program seg0 pre_solve failed");
        require(std::abs(st->pedaling.effort - 32.0f) <= 1e-3f &&
                    st->last_brakes.final_front == 0.0f,
                "RideProgram step segment must override human_torque_nm without braking");

        // Segment 1 linear ramp midpoint at t = 0.015 s (step_index = 12).
        st->step_index = 12;
        require(bike_gpu_launch_sp3_pre_solve(
                    nullptr, 1, kDtS, &idx13, &tt13->tyre, &tt13->terrain_desc,
                    tt13->terrain_f32.data(), tt13->terrain_i32.data(),
                    cfg.get(), 0, st.get(), ws.get(),
                    b.qpos.data(), b.qvel.data(), b.xpos.data(), b.xmat.data(),
                    b.xanchor.data(), b.xaxis.data(), b.qfrc.data(), b.ctrl.data(),
                    nt13, b.tendon_range.data(), 2 * nt13,
                    b.dof_frictionloss.data(), nv13) == 0,
                "ride_program seg1 ramp pre_solve failed");
        require(std::abs(st->pedaling.effort - 42.0f) <= 1e-2f &&
                    st->last_brakes.final_front == 0.0f,
                "RideProgram linear_ramp segment must interpolate human_torque_nm to midpoint (42 Nm)");

        // Segment 2 braking segment at t = 0.025 s (step_index = 20): front_brake > 0 forces legs_coast.
        st->step_index = 20;
        require(bike_gpu_launch_sp3_pre_solve(
                    nullptr, 1, kDtS, &idx13, &tt13->tyre, &tt13->terrain_desc,
                    tt13->terrain_f32.data(), tt13->terrain_i32.data(),
                    cfg.get(), 0, st.get(), ws.get(),
                    b.qpos.data(), b.qvel.data(), b.xpos.data(), b.xmat.data(),
                    b.xanchor.data(), b.xaxis.data(), b.qfrc.data(), b.ctrl.data(),
                    nt13, b.tendon_range.data(), 2 * nt13,
                    b.dof_frictionloss.data(), nv13) == 0,
                "ride_program seg2 brake pre_solve failed");
        require(std::abs(st->last_brakes.final_front - 0.25f) <= 1e-4f &&
                    st->pedaling.effort == 0.0f,
                "RideProgram braking segment must set final_front=0.25 and coast legs (effort=0)");
    }
    std::cout << "[test_sp3_cruise_and_ride_program] passed\n";
}

[[gnu::noinline]] void test_sp3_decision11_static_brake(const TempSp3Models &models) {
    ModelPtr m13 = load_xml(models.xml13);
    const auto idx13 = resolve_model_indices(m13.get());
    const auto tt13 = build_tyre_terrain_from_xml(models.xml13, idx13);
    const int nv13 = to_int(m13->nv);
    const int nt13 = to_int(m13->ntendon);

    const auto cfg = build_sp3_config_for_model(m13.get());
    cfg->intent.enabled = 0;
    cfg->shifting.enabled = 0;
    cfg->static_brake_ceiling_nm = 200.0f;
    cfg->brake_hold.torque_nm = 200.0f;
    cfg->brake_hold.stiffness_nm_rad = 20000.0f;
    cfg->brake_hold.damping_nms_rad = 10.0f;
    cfg->caller_front_brake = 1.0f;
    cfg->caller_rear_brake = 0.5f;

    DataPtr d(nullptr, &mj_deleteData);
    build_riding_initial_state(m13.get(), idx13, {.speed_mps = 0.0, .crank_omega_rad_s = 0.0}, d);
    auto b = snapshot_mj_to_float(m13.get(), d.get());
    const auto st = std::make_unique<bike::gpu::GpuSp3WorldState>();
    const auto ws = std::make_unique<bike::gpu::GpuWorldState>();
    require(bike_gpu_launch_sp3_init(
                nullptr, 1, &idx13, cfg.get(), 0, st.get(), ws.get(),
                b.qpos.data(), 0.0f, nt13, b.tendon_range.data(), 2 * nt13) == 0,
            "static_brake init failed");

    // Step 0 engages brake hold latches at standstill.
    require(bike_gpu_launch_sp3_pre_solve(
                nullptr, 1, kDtS, &idx13, &tt13->tyre, &tt13->terrain_desc,
                tt13->terrain_f32.data(), tt13->terrain_i32.data(),
                cfg.get(), 0, st.get(), ws.get(),
                b.qpos.data(), b.qvel.data(), b.xpos.data(), b.xmat.data(),
                b.xanchor.data(), b.xaxis.data(), b.qfrc.data(), b.ctrl.data(),
                nt13, b.tendon_range.data(), 2 * nt13,
                b.dof_frictionloss.data(), nv13) == 0,
            "static_brake step 0 failed");
    require(st->sp3.monitors.brake.latch_front != 0 && st->sp3.monitors.brake.latch_rear != 0,
            "Decision 11 brake hold latches must engage at standstill");

    // Displace wheel angles while latched so hold_nm > 0 and verify dof_frictionloss is reduced by |hold_nm|.
    b.qpos[to_index(idx13.front_wheel_spin_qpos)] += 0.005f; // |hold| ~ 100 Nm
    b.qpos[to_index(idx13.rear_wheel_spin_qpos)] += 0.002f;  // |hold| ~ 40 Nm
    require(bike_gpu_launch_sp3_pre_solve(
                nullptr, 1, kDtS, &idx13, &tt13->tyre, &tt13->terrain_desc,
                tt13->terrain_f32.data(), tt13->terrain_i32.data(),
                cfg.get(), 0, st.get(), ws.get(),
                b.qpos.data(), b.qvel.data(), b.xpos.data(), b.xmat.data(),
                b.xanchor.data(), b.xaxis.data(), b.qfrc.data(), b.ctrl.data(),
                nt13, b.tendon_range.data(), 2 * nt13,
                b.dof_frictionloss.data(), nv13) == 0,
            "static_brake step 1 failed");

    const float front_hold = std::abs(st->last_front_hold_nm);
    const float rear_hold = std::abs(st->last_rear_hold_nm);
    const float front_fl = b.dof_frictionloss[to_index(idx13.front_wheel_spin_dof)];
    const float rear_fl = b.dof_frictionloss[to_index(idx13.rear_wheel_spin_dof)];
    require(front_hold > 90.0f && rear_hold > 30.0f,
            "displaced latched wheels must produce non-trivial hold torque");
    require(front_hold + front_fl <= cfg->static_brake_ceiling_nm * 1.0f + 1e-3f &&
                std::abs((front_hold + front_fl) - 200.0f) <= 1e-2f,
            "Decision 11 front dof_frictionloss must equal max(0, ceiling*demand - |hold_nm|)");
    require(rear_hold + rear_fl <= cfg->static_brake_ceiling_nm * 0.5f + 1e-3f &&
                std::abs((rear_hold + rear_fl) - 100.0f) <= 1e-2f,
            "Decision 11 rear dof_frictionloss must equal max(0, ceiling*demand - |hold_nm|)");
    std::cout << "[test_sp3_decision11_static_brake] front=(" << front_hold << "+" << front_fl
              << ") rear=(" << rear_hold << "+" << rear_fl << ")\n";
}

[[gnu::noinline]] void test_sp3_stall_control_clock_and_freeze_on_crash(const TempSp3Models &models) {
    ModelPtr m13 = load_xml(models.xml13);
    const auto idx13 = resolve_model_indices(m13.get());
    const auto tt13 = build_tyre_terrain_from_xml(models.xml13, idx13);
    const int nv13 = to_int(m13->nv);
    const int nt13 = to_int(m13->ntendon);
    const int nsd13 = to_int(m13->nsensordata);

    // 1. StallDetector / reposition_on_stall
    {
        const auto cfg = build_sp3_config_for_model(m13.get());
        cfg->apply_direct_crank_torque = 0;
        cfg->intent.enabled = 0;
        cfg->shifting.enabled = 0;
        cfg->caller_human_torque_nm = 30.0f;
        cfg->pedaling.reposition_on_stall = 1;
        cfg->pedaling.reposition_min_effort_nm = 20.0f;
        cfg->pedaling.reposition_stall_window_ticks = 6;
        cfg->pedaling.reposition_stall_progress_rad = 0.5f;

        DataPtr d(nullptr, &mj_deleteData);
        build_riding_initial_state(m13.get(), idx13, {.speed_mps = 0.0, .crank_omega_rad_s = 0.0}, d);
        auto b = snapshot_mj_to_float(m13.get(), d.get());
        b.qpos[to_index(idx13.crank_spin_qpos)] = 0.7f;
        const auto st = std::make_unique<bike::gpu::GpuSp3WorldState>();
        const auto ws = std::make_unique<bike::gpu::GpuWorldState>();
        require(bike_gpu_launch_sp3_init(
                    nullptr, 1, &idx13, cfg.get(), 0, st.get(), ws.get(),
                    b.qpos.data(), 0.0f, nt13, b.tendon_range.data(), 2 * nt13) == 0,
                "stall init failed");

        for (int s = 0; s < 10; ++s) {
            st->step_index = s;
            require(bike_gpu_launch_sp3_pre_solve(
                        nullptr, 1, kDtS, &idx13, &tt13->tyre, &tt13->terrain_desc,
                        tt13->terrain_f32.data(), tt13->terrain_i32.data(),
                        cfg.get(), 0, st.get(), ws.get(),
                        b.qpos.data(), b.qvel.data(), b.xpos.data(), b.xmat.data(),
                        b.xanchor.data(), b.xaxis.data(), b.qfrc.data(), b.ctrl.data(),
                        nt13, b.tendon_range.data(), 2 * nt13,
                        b.dof_frictionloss.data(), nv13) == 0,
                    "stall pre_solve failed");
        }
        require(st->stall_reposition.has_goal == 1 && st->stall_reposition.reason == 2,
                "StallDetector with reposition_on_stall=1 must trigger stall reposition goal (reason=2)");
    }

    // 2. ControlClock (control_period_steps > 1) + freeze_on_crash = 1
    {
        const auto cfg = build_sp3_config_for_model(m13.get());
        cfg->intent.enabled = 0;
        cfg->shifting.enabled = 0;
        cfg->control_period_steps = 4;
        cfg->cruise_enabled = 1;
        cfg->cruise.target_speed_kmh = 30.0f;
        cfg->cruise.kp_nm_per_mps = 40.0f;
        cfg->cruise.ki_nm_per_mps_s = 10.0f;
        cfg->cruise.torque_ceiling_nm = 120.0f;
        cfg->freeze_on_crash = 1;

        DataPtr d(nullptr, &mj_deleteData);
        build_riding_initial_state(m13.get(), idx13, {.speed_mps = 4.0, .crank_omega_rad_s = 6.0}, d);
        auto b = snapshot_mj_to_float(m13.get(), d.get());
        const auto st = std::make_unique<bike::gpu::GpuSp3WorldState>();
        const auto ws = std::make_unique<bike::gpu::GpuWorldState>();
        require(bike_gpu_launch_sp3_init(
                    nullptr, 1, &idx13, cfg.get(), 0, st.get(), ws.get(),
                    b.qpos.data(), 0.0f, nt13, b.tendon_range.data(), 2 * nt13) == 0,
                "clock/freeze init failed");
        st->last_wheel_grounded[1] = 1;
        st->sp3.monitors.grounded[1].value = 1;

        st->step_index = 0;
        require(bike_gpu_launch_sp3_pre_solve(
                    nullptr, 1, kDtS, &idx13, &tt13->tyre, &tt13->terrain_desc,
                    tt13->terrain_f32.data(), tt13->terrain_i32.data(),
                    cfg.get(), 0, st.get(), ws.get(),
                    b.qpos.data(), b.qvel.data(), b.xpos.data(), b.xmat.data(),
                    b.xanchor.data(), b.xaxis.data(), b.qfrc.data(), b.ctrl.data(),
                    nt13, b.tendon_range.data(), 2 * nt13,
                    b.dof_frictionloss.data(), nv13) == 0,
                "control_clock step 0 failed");
        const float tq0 = st->last_cruise_torque_nm;
        require(tq0 > 1.0f, "control_clock step 0 cruise torque must be positive");

        b.qvel[to_index(idx13.root_x_dof)] = 6.5f;
        for (int s = 1; s <= 3; ++s) {
            st->step_index = s;
            require(bike_gpu_launch_sp3_pre_solve(
                        nullptr, 1, kDtS, &idx13, &tt13->tyre, &tt13->terrain_desc,
                        tt13->terrain_f32.data(), tt13->terrain_i32.data(),
                        cfg.get(), 0, st.get(), ws.get(),
                        b.qpos.data(), b.qvel.data(), b.xpos.data(), b.xmat.data(),
                        b.xanchor.data(), b.xaxis.data(), b.qfrc.data(), b.ctrl.data(),
                        nt13, b.tendon_range.data(), 2 * nt13,
                        b.dof_frictionloss.data(), nv13) == 0,
                    "control_clock off-tick failed");
            require(st->last_cruise_torque_nm == tq0,
                    "ControlClock must hold cruise torque between control_period_steps=4 ticks");
        }
        st->step_index = 4;
        require(bike_gpu_launch_sp3_pre_solve(
                    nullptr, 1, kDtS, &idx13, &tt13->tyre, &tt13->terrain_desc,
                    tt13->terrain_f32.data(), tt13->terrain_i32.data(),
                    cfg.get(), 0, st.get(), ws.get(),
                    b.qpos.data(), b.qvel.data(), b.xpos.data(), b.xmat.data(),
                    b.xanchor.data(), b.xaxis.data(), b.qfrc.data(), b.ctrl.data(),
                    nt13, b.tendon_range.data(), 2 * nt13,
                    b.dof_frictionloss.data(), nv13) == 0,
                "control_clock tick 4 failed");
        require(st->last_cruise_torque_nm < tq0,
                "ControlClock must update cruise torque on step_index=4");

        // Trigger pitch_over crash -> freeze_on_crash = 1 freezes world state.
        b.qpos[to_index(idx13.root_pitch_qpos)] = 2.0f;
        require(bike_gpu_launch_sp3_post_solve(
                    nullptr, 1, kDtS, &idx13, cfg.get(), 0, st.get(),
                    b.qpos.data(), b.qvel.data(), b.xmat.data(),
                    nsd13, b.sensordata.data(),
                    b.nefc.data(), 1, nv13,
                    b.efc_type.data(), b.efc_id.data(), b.efc_J.data(), b.efc_force.data(),
                    b.nacon.data(), 1, 4,
                    b.contact_worldid.data(), b.contact_geom.data(),
                    b.contact_efc_address.data(), b.contact_dim.data()) == 0,
                "crash post_solve failed");
        require(st->frozen == 1 && bike_gpu_sp3_state_has_failure(st.get()) == 1,
                "freeze_on_crash=1 must latch frozen=1 on pitch_over");
        const int frozen_step = st->step_index;
        std::ranges::fill(b.qfrc, 7.5f);
        std::ranges::fill(b.ctrl, 3.5f);
        require(bike_gpu_launch_sp3_pre_solve(
                    nullptr, 1, kDtS, &idx13, &tt13->tyre, &tt13->terrain_desc,
                    tt13->terrain_f32.data(), tt13->terrain_i32.data(),
                    cfg.get(), 0, st.get(), ws.get(),
                    b.qpos.data(), b.qvel.data(), b.xpos.data(), b.xmat.data(),
                    b.xanchor.data(), b.xaxis.data(), b.qfrc.data(), b.ctrl.data(),
                    nt13, b.tendon_range.data(), 2 * nt13,
                    b.dof_frictionloss.data(), nv13) == 0,
                "frozen pre_solve failed");
        for (const float v : b.qfrc) {
            require(v == 0.0f, "frozen pre_solve must zero qfrc");
        }
        for (const float v : b.ctrl) {
            require(v == 0.0f, "frozen pre_solve must zero ctrl");
        }
        require(bike_gpu_launch_sp3_post_solve(
                    nullptr, 1, kDtS, &idx13, cfg.get(), 0, st.get(),
                    b.qpos.data(), b.qvel.data(), b.xmat.data(),
                    nsd13, b.sensordata.data(),
                    b.nefc.data(), 1, nv13,
                    b.efc_type.data(), b.efc_id.data(), b.efc_J.data(), b.efc_force.data(),
                    b.nacon.data(), 1, 4,
                    b.contact_worldid.data(), b.contact_geom.data(),
                    b.contact_efc_address.data(), b.contact_dim.data()) == 0,
                "frozen post_solve failed");
        require(st->step_index == frozen_step,
                "frozen post_solve must keep step_index unchanged");
    }
    std::cout << "[test_sp3_stall_control_clock_and_freeze_on_crash] passed\n";
}

} // namespace

int main() {
    try {
        const TempSp3Models models(repo_root() / "tests/fixtures/model/welded.xml");
        test_12tendon_steady_gear_and_shift(models);
        test_post_solve_efc_reaction_parity(models);
        test_2pi_epoch_rebasing_invariance(models);
        test_host_vs_metal_and_multiworld_isolation(models);
        test_sp3_assist_coast_gate_and_launch_preload(models);
        test_sp3_cruise_and_ride_program(models);
        test_sp3_decision11_static_brake(models);
        test_sp3_stall_control_clock_and_freeze_on_crash(models);
        std::cout << "[bike_test_gpu_sp3_autonomous] ALL PASSED\n";
        return 0;
    } catch (const std::exception &ex) {
        std::cerr << "[bike_test_gpu_sp3_autonomous] FAILED: " << ex.what() << '\n';
        return 1;
    }
}
