// native/tests/gpu/test_gpu_batch_runners.cpp — C++23 integration and parity test
// for cpu_batch_runner.cpp, cuda_kernels.cu (host ABI), and metal_batch_runner.mm
// across both compliant_2d and distributed_2d_reference tyre backends.

#include "../../src/gpu/batch_step.hpp"
#include "../../src/gpu/cuda_kernels_abi.hpp"
#include "../../src/gpu/model_assets.hpp"
#include "../../src/gpu/runner_tyre_setup.hpp"
#include "../../src/gpu/terrain_tables.hpp"
#include "../../src/model_access.hpp"
#include "../../src/platform/argv.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
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

int bike_cpu_run_batch_sweep(
    const char *xml_path,
    int nworld,
    int n_steps,
    int num_threads,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::physics_core::GpuTyreModel *d_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *d_terrain_desc,
    const float *d_terrain_f32,
    const bike::physics_core::i32 *d_terrain_i32,
    const bike::gpu::GpuWorldConfig *d_configs,
    int per_world_config,
    const double *init_qpos,
    const double *init_qvel,
    float *out_peak_fork_mm,
    float *out_peak_shock_mm,
    float *out_final_root_x_m,
    int *out_all_finite,
    double *out_elapsed_ms
);

#if defined(__APPLE__)
int bike_metal_init(const char *gpu_src_dir);
const char *bike_metal_device_name();

int bike_metal_launch_init_states(
    int nworld,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::gpu::GpuWorldConfig *d_configs,
    int per_world_config,
    bike::gpu::GpuWorldState *d_states,
    const float *d_qpos
);

int bike_metal_launch_writers(
    int nworld,
    float dt_s,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::physics_core::GpuTyreModel *d_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *d_terrain_desc,
    const float *d_terrain_f32,
    const bike::physics_core::i32 *d_terrain_i32,
    const bike::gpu::GpuWorldConfig *d_configs,
    int per_world_config,
    bike::gpu::GpuWorldState *d_states,
    const float *d_qpos,
    const float *d_qvel,
    const float *d_xpos,
    const float *d_xmat,
    const float *d_xanchor,
    const float *d_xaxis,
    float *d_qfrc_applied,
    float *d_ctrl,
    float *d_tendon_range,
    int tendon_range_world_stride
);

int bike_metal_run_hybrid_sweep(
    const char *xml_path,
    int nworld,
    int n_steps,
    int num_threads,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::physics_core::GpuTyreModel *d_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *d_terrain_desc,
    const float *d_terrain_f32,
    const bike::physics_core::i32 *d_terrain_i32,
    const bike::gpu::GpuWorldConfig *d_configs,
    int per_world_config,
    const double *init_qpos,
    const double *init_qvel,
    float *out_peak_fork_mm,
    float *out_peak_shock_mm,
    float *out_final_root_x_m,
    int *out_all_finite,
    double *out_elapsed_ms
);
#endif

} // extern "C"

namespace {

namespace fs = std::filesystem;
using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPtr = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

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

void require_above(double actual, double bound, std::string_view what) {
    require(actual > bound,
            std::string(what) + ": actual=" + std::to_string(actual) +
                " bound=" + std::to_string(bound));
}

void require_below(double actual, double bound, std::string_view what) {
    require(actual < bound,
            std::string(what) + ": actual=" + std::to_string(actual) +
                " bound=" + std::to_string(bound));
}

constexpr std::size_t to_index(int field) {
    return static_cast<std::size_t>(field);
}

constexpr const char *kRelModelPath = "tools/fp32_probe/artifacts/common/model.xml";
constexpr const char *kRelWeldedPath = "tests/fixtures/model/welded.xml";

fs::path find_repo_file(const char *rel_path) {
    std::vector<fs::path> candidates;
#ifdef BIKE_REPO_ROOT
    candidates.emplace_back(fs::path(BIKE_REPO_ROOT) / rel_path);
#endif
    candidates.emplace_back(rel_path);
    candidates.emplace_back(fs::path("..") / rel_path);
    candidates.emplace_back(fs::path("../..") / rel_path);
    candidates.emplace_back(fs::path("../../..") / rel_path);
    for (const auto &p : candidates) {
        std::error_code ec;
        if (fs::is_regular_file(p, ec)) {
            return fs::canonical(p, ec);
        }
    }
    return {};
}

ModelPtr load_xml(const fs::path &xml_path) {
    std::array<char, 1024> mj_err{};
    mjModel *raw = mj_loadXML(xml_path.string().c_str(), nullptr,
                              mj_err.data(), static_cast<int>(mj_err.size()));
    require(raw != nullptr, std::string("mj_loadXML failed: ") + mj_err.data());
    return {raw, &mj_deleteModel};
}

void seed_flat_welded_road(const mjModel *m) {
    const int hf_id = mj_name2id(m, mjOBJ_HFIELD, "road");
    const int terrain_geom = mj_name2id(m, mjOBJ_GEOM, "terrain");
    require(hf_id >= 0 && terrain_geom >= 0, "welded.xml requires road hfield and terrain geom");
    const std::span<const int> hfield_nrow =
        model_access::readonly_buffer(m->hfield_nrow, m->nhfield, "hfield_nrow");
    const std::span<const int> hfield_ncol =
        model_access::readonly_buffer(m->hfield_ncol, m->nhfield, "hfield_ncol");
    const std::span<const int> hfield_adr =
        model_access::readonly_buffer(m->hfield_adr, m->nhfield, "hfield_adr");
    const std::span<const mjtNum> hfield_size =
        model_access::readonly_buffer(m->hfield_size, 4 * m->nhfield, "hfield_size");
    const std::span<const mjtNum> geom_pos =
        model_access::readonly_buffer(m->geom_pos, 3 * m->ngeom, "geom_pos");
    const int nrow = hfield_nrow[to_index(hf_id)];
    const int ncol = hfield_ncol[to_index(hf_id)];
    const int adr = hfield_adr[to_index(hf_id)];
    const double sz = static_cast<double>(hfield_size[4 * to_index(hf_id) + 2]);
    const double pos_z = static_cast<double>(geom_pos[3 * to_index(terrain_geom) + 2]);
    const float flat_norm = static_cast<float>((0.0 - pos_z) / sz);
    const std::span<float> hf =
        model_access::mutable_buffer(m->hfield_data, m->nhfielddata, "hfield_data");
    for (int r = 0; r < nrow; ++r) {
        for (int c = 0; c < ncol; ++c) {
            hf[to_index(adr + r * ncol + c)] = flat_norm;
        }
    }
}

struct TempWeldedAssetDir {
    fs::path dir;
    fs::path xml_copy;

    TempWeldedAssetDir(
        const fs::path &welded_src,
        const mjModel *m,
        std::string_view tag
    ) {
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        dir = fs::temp_directory_path() / std::format("bike_sp1_{}_{}", tag, stamp);
        fs::create_directories(dir);
        xml_copy = dir / "welded.xml";
        fs::copy_file(welded_src, xml_copy, fs::copy_options::overwrite_existing);

        if (m->nhfielddata > 0) {
            const fs::path bin_path = dir / "hfield_data.bin";
            std::ofstream ofs(bin_path, std::ios::binary);
            require(ofs.is_open(), "open temp hfield_data.bin for writing");
            const std::span<const float> hf =
                model_access::readonly_buffer(m->hfield_data, m->nhfielddata, "hfield_data");
            for (const float v : hf) {
                const double d = static_cast<double>(v);
                const auto bytes = std::bit_cast<std::array<char, sizeof(double)>>(d);
                ofs.write(bytes.data(), sizeof(double));
            }
            require(ofs.good(), "write temp hfield_data.bin");
        }
    }

    TempWeldedAssetDir(const TempWeldedAssetDir &) = delete;
    TempWeldedAssetDir &operator=(const TempWeldedAssetDir &) = delete;
    TempWeldedAssetDir(TempWeldedAssetDir &&) = delete;
    TempWeldedAssetDir &operator=(TempWeldedAssetDir &&) = delete;

    ~TempWeldedAssetDir() {
        if (!dir.empty()) {
            std::error_code ec;
            fs::remove_all(dir, ec);
        }
    }
};

void build_welded_initial_state(
    const mjModel *m,
    const bike::gpu::GpuModelIndices &idx,
    std::vector<double> &qpos0,
    std::vector<double> &qvel0,
    DataPtr &out_data
) {
    DataPtr d(mj_makeData(m), &mj_deleteData);
    mj_resetData(m, d.get());

    const int nq = idx.nq;
    const int nv = idx.nv;
    const int rider_rx_jnt = mj_name2id(m, mjOBJ_JOINT, "rider_root_x");
    const int rider_rz_jnt = mj_name2id(m, mjOBJ_JOINT, "rider_root_z");
    require(rider_rx_jnt >= 0 && rider_rz_jnt >= 0, "welded.xml must have rider_root_x/z");
    const std::span<const int> jnt_qposadr =
        model_access::readonly_buffer(m->jnt_qposadr, m->njnt, "jnt_qposadr");
    const std::span<const int> jnt_dofadr =
        model_access::readonly_buffer(m->jnt_dofadr, m->njnt, "jnt_dofadr");
    const int rider_rx_qpos = jnt_qposadr[to_index(rider_rx_jnt)];
    const int rider_rz_qpos = jnt_qposadr[to_index(rider_rz_jnt)];
    const int rider_rx_dof = jnt_dofadr[to_index(rider_rx_jnt)];

    const std::span<mjtNum> d_qpos = model_access::mutable_buffer(d->qpos, nq, "qpos");
    const std::span<mjtNum> d_qvel = model_access::mutable_buffer(d->qvel, nv, "qvel");
    d_qpos[to_index(idx.root_x_qpos)] = 2.0;
    d_qpos[to_index(rider_rx_qpos)] = 2.0;
    mj_forward(m, d.get());

    const int f_body = idx.front_wheel_body;
    const int r_body = idx.rear_wheel_body;
    const int f_geom = bike::gpu::find_wheel_contact_geom(m, f_body, "front_contact", "geom_front_contact");
    const int r_geom = bike::gpu::find_wheel_contact_geom(m, r_body, "rear_contact", "geom_rear_contact");
    require(f_geom >= 0 && r_geom >= 0, "welded.xml wheel contact geoms resolved");
    const std::span<const mjtNum> geom_size =
        model_access::readonly_buffer(m->geom_size, 3 * m->ngeom, "geom_size");
    const std::span<const mjtNum> d_xpos_cur =
        model_access::readonly_buffer(d->xpos, 3LL * static_cast<mjtSize>(idx.nbody), "xpos");
    const double f_r = geom_size[3 * to_index(f_geom)];
    const double r_r = geom_size[3 * to_index(r_geom)];
    const double min_clear = std::min(d_xpos_cur[3 * to_index(f_body) + 2] - f_r,
                                      d_xpos_cur[3 * to_index(r_body) + 2] - r_r);
    const double dz = -0.015 - min_clear;
    d_qpos[to_index(idx.root_z_qpos)] += dz;
    d_qpos[to_index(rider_rz_qpos)] += dz;
    d_qvel[to_index(idx.root_x_dof)] = 5.0;
    d_qvel[to_index(rider_rx_dof)] = 5.0;
    d_qvel[to_index(idx.front_wheel_spin_dof)] = 5.0 / f_r;
    d_qvel[to_index(idx.rear_wheel_spin_dof)] = 5.0 / r_r;
    d_qvel[to_index(idx.crank_spin_dof)] = 6.0;
    if (idx.drive_shaft_spin_dof >= 0) {
        d_qvel[to_index(idx.drive_shaft_spin_dof)] = 6.0;
    }
    mj_forward(m, d.get());

    qpos0.resize(to_index(nq));
    qvel0.resize(to_index(nv));
    for (int i = 0; i < nq; ++i) {
        qpos0[to_index(i)] = static_cast<double>(d_qpos[to_index(i)]);
    }
    for (int i = 0; i < nv; ++i) {
        qvel0[to_index(i)] = static_cast<double>(d_qvel[to_index(i)]);
    }
    out_data = std::move(d);
}

#if defined(__APPLE__)
void verify_metal_vs_cpu_writers_parity(
    const ModelPtr &m,
    const bike::gpu::GpuModelIndices &idx,
    const nativecfg::TireConfig &tcfg,
    std::string_view label
) {
    using namespace bike::gpu;
    using namespace bike::physics_core;

    DataPtr d(mj_makeData(m.get()), &mj_deleteData);
    mj_resetData(m.get(), d.get());

    const int nq = idx.nq;
    const int nv = idx.nv;
    const int nu = idx.nu;
    const int nbody = idx.nbody;
    const int njnt = static_cast<int>(m->njnt);

    const std::span<mjtNum> d_qpos = model_access::mutable_buffer(d->qpos, nq, "qpos");
    const std::span<mjtNum> d_qvel = model_access::mutable_buffer(d->qvel, nv, "qvel");
    d_qpos[to_index(idx.root_x_qpos)] = 2.0;
    d_qpos[to_index(idx.root_z_qpos)] = 0.35;
    d_qpos[to_index(idx.fork_travel_qpos)] = 0.045;
    d_qpos[to_index(idx.shock_stroke_qpos)] = 0.020;
    mj_forward(m.get(), d.get());

    const int f_body = idx.front_wheel_body;
    const int r_body = idx.rear_wheel_body;
    const int f_geom = find_wheel_contact_geom(m.get(), f_body, "front_contact", "geom_front_contact");
    const int r_geom = find_wheel_contact_geom(m.get(), r_body, "rear_contact", "geom_rear_contact");
    require(f_geom >= 0 && r_geom >= 0, "welded.xml wheel contact geoms resolved");
    const std::span<const mjtNum> geom_size =
        model_access::readonly_buffer(m->geom_size, 3 * m->ngeom, "geom_size");
    const std::span<const mjtNum> d_xpos_cur =
        model_access::readonly_buffer(d->xpos, 3LL * static_cast<mjtSize>(nbody), "xpos");
    const double f_r = geom_size[3 * to_index(f_geom)];
    const double r_r = geom_size[3 * to_index(r_geom)];
    const double min_clear = std::min(d_xpos_cur[3 * to_index(f_body) + 2] - f_r,
                                      d_xpos_cur[3 * to_index(r_body) + 2] - r_r);
    d_qpos[to_index(idx.root_z_qpos)] += (-0.015 - min_clear);
    d_qvel[to_index(idx.root_x_dof)] = 6.0;
    d_qvel[to_index(idx.root_z_dof)] = -0.10;
    d_qvel[to_index(idx.front_wheel_spin_dof)] = 6.0 / f_r + 2.5;
    d_qvel[to_index(idx.rear_wheel_spin_dof)] = 6.0 / r_r - 2.0;
    d_qvel[to_index(idx.crank_spin_dof)] = 8.0;
    mj_forward(m.get(), d.get());

    const auto bundle_ptr = std::make_unique<HostTyreUploadBundle>(build_gpu_tyre_model(
        m.get(), d.get(), tcfg, f_geom, r_geom, f_body, r_body));
    const HostTyreUploadBundle &bundle = *bundle_ptr;
    const HostPackedTerrain terrain = pack_upload_bundle_terrain(bundle);

    constexpr int kNWorld = 8;
    std::vector<float> fork_scale(to_index(kNWorld));
    std::vector<float> reb_scale(to_index(kNWorld));
    std::vector<float> shock_scale(to_index(kNWorld));
    std::vector<float> tire_scale(to_index(kNWorld));
    for (int w = 0; w < kNWorld; ++w) {
        const float u = static_cast<float>(w) / static_cast<float>(kNWorld - 1);
        fork_scale[to_index(w)] = 0.85f + 0.30f * u;
        reb_scale[to_index(w)] = 0.80f + 0.40f * u;
        shock_scale[to_index(w)] = 0.90f + 0.25f * u;
        tire_scale[to_index(w)] = 0.85f + 0.35f * u;
    }
    std::vector<GpuWorldConfig> configs(to_index(kNWorld));
    require(bike_gpu_build_sweep_configs(
                configs.data(), kNWorld, fork_scale.data(), reb_scale.data(),
                shock_scale.data(), tire_scale.data()) == 0,
            "bike_gpu_build_sweep_configs succeeds");

    std::vector<float> qpos_batch(to_index(kNWorld * nq), 0.0f);
    std::vector<float> qvel_batch(to_index(kNWorld * nv), 0.0f);
    std::vector<float> xpos_batch(to_index(kNWorld * nbody * 3), 0.0f);
    std::vector<float> xmat_batch(to_index(kNWorld * nbody * 9), 0.0f);
    std::vector<float> xanchor_batch(to_index(kNWorld * njnt * 3), 0.0f);
    std::vector<float> xaxis_batch(to_index(kNWorld * njnt * 3), 0.0f);

    const std::span<const mjtNum> src_xpos = model_access::readonly_buffer(d->xpos, static_cast<mjtSize>(nbody) * 3LL);
    const std::span<const mjtNum> src_xmat = model_access::readonly_buffer(d->xmat, static_cast<mjtSize>(nbody) * 9LL);
    const std::span<const mjtNum> src_xanc = model_access::readonly_buffer(d->xanchor, static_cast<mjtSize>(njnt) * 3LL);
    const std::span<const mjtNum> src_xax = model_access::readonly_buffer(d->xaxis, static_cast<mjtSize>(njnt) * 3LL);
    for (int w = 0; w < kNWorld; ++w) {
        const std::size_t wu = to_index(w);
        for (int i = 0; i < nq; ++i) {
            qpos_batch[wu * to_index(nq) + to_index(i)] = static_cast<float>(d_qpos[to_index(i)]);
        }
        for (int i = 0; i < nv; ++i) {
            qvel_batch[wu * to_index(nv) + to_index(i)] = static_cast<float>(d_qvel[to_index(i)]);
        }
        for (int i = 0; i < nbody * 3; ++i) {
            xpos_batch[wu * to_index(nbody * 3) + to_index(i)] = static_cast<float>(src_xpos[to_index(i)]);
        }
        for (int i = 0; i < nbody * 9; ++i) {
            xmat_batch[wu * to_index(nbody * 9) + to_index(i)] = static_cast<float>(src_xmat[to_index(i)]);
        }
        for (int i = 0; i < njnt * 3; ++i) {
            xanchor_batch[wu * to_index(njnt * 3) + to_index(i)] = static_cast<float>(src_xanc[to_index(i)]);
            xaxis_batch[wu * to_index(njnt * 3) + to_index(i)] = static_cast<float>(src_xax[to_index(i)]);
        }
    }

    std::vector<GpuWorldState> st_mtl(to_index(kNWorld));
    std::vector<GpuWorldState> st_cpu(to_index(kNWorld));
    require(bike_metal_launch_init_states(kNWorld, &idx, configs.data(), 1, st_mtl.data(), qpos_batch.data()) == 0,
            "metal init_states succeeds");
    require(bike_gpu_launch_init_states(nullptr, kNWorld, &idx, configs.data(), 1, st_cpu.data(), qpos_batch.data()) == 0,
            "cpu init_states succeeds");

    std::vector<float> qfrc_mtl(to_index(kNWorld * nv), 0.0f);
    std::vector<float> qfrc_cpu(to_index(kNWorld * nv), 0.0f);
    std::vector<float> ctrl_mtl(to_index(kNWorld * nu), 0.0f);
    std::vector<float> ctrl_cpu(to_index(kNWorld * nu), 0.0f);
    std::vector<float> tendon_mtl(to_index(kNWorld) * 4U, 0.0f);
    std::vector<float> tendon_cpu(to_index(kNWorld) * 4U, 0.0f);

    for (int tick = 0; tick < 3; ++tick) {
        const float dt_s = 0.001f;
        const float *tf32 = terrain.f32.empty() ? nullptr : terrain.f32.data();
        const i32 *ti32 = terrain.i32.empty() ? nullptr : terrain.i32.data();
        require(bike_metal_launch_writers(
                    kNWorld, dt_s, &idx, &bundle.gpu_model, &terrain.desc, tf32, ti32,
                    configs.data(), 1, st_mtl.data(),
                    qpos_batch.data(), qvel_batch.data(),
                    xpos_batch.data(), xmat_batch.data(), xanchor_batch.data(), xaxis_batch.data(),
                    qfrc_mtl.data(), ctrl_mtl.data(), tendon_mtl.data(), 4) == 0,
                std::format("metal launch_writers succeeds ({} tick {})", label, tick));
        require(bike_gpu_launch_writers(
                    nullptr, kNWorld, dt_s, &idx, &bundle.gpu_model, &terrain.desc, tf32, ti32,
                    configs.data(), 1, st_cpu.data(),
                    qpos_batch.data(), qvel_batch.data(),
                    xpos_batch.data(), xmat_batch.data(), xanchor_batch.data(), xaxis_batch.data(),
                    qfrc_cpu.data(), ctrl_cpu.data(), tendon_cpu.data(), 4) == 0,
                std::format("cpu launch_writers succeeds ({} tick {})", label, tick));

        float max_qfrc = 0.0f;
        for (const float v : qfrc_cpu) {
            max_qfrc = std::max(max_qfrc, std::abs(v));
        }
        require(max_qfrc > 100.0f, "contact writers produce strong ground reaction force");
        const float qfrc_tol = 2e-5f * std::max(1.0f, max_qfrc);
        for (std::size_t i = 0; i < qfrc_cpu.size(); ++i) {
            require_near(qfrc_mtl[i], qfrc_cpu[i], qfrc_tol,
                         std::format("Metal vs CPU qfrc parity ({} tick {} slot {})",
                                     label, tick, i));
        }
        for (std::size_t i = 0; i < ctrl_cpu.size(); ++i) {
            const float c_tol = 2e-5f * std::max(1.0f, std::abs(ctrl_cpu[i]));
            require_near(ctrl_mtl[i], ctrl_cpu[i], c_tol,
                         std::format("Metal vs CPU ctrl parity ({} tick {})", label, tick));
        }
        for (int w = 0; w < kNWorld; ++w) {
            const std::size_t wu = to_index(w);
            require_near(tendon_mtl[4U * wu + 1U], tendon_cpu[4U * wu + 1U], 2e-5f,
                         std::format("Metal vs CPU freehub tendon bound ({} tick {})", label, tick));
            require_near(tendon_mtl[4U * wu + 3U], tendon_cpu[4U * wu + 3U], 2e-5f,
                         std::format("Metal vs CPU crank clutch tendon bound ({} tick {})", label, tick));
            require_near(st_mtl[wu].drivetrain.freehub.boundary,
                         st_cpu[wu].drivetrain.freehub.boundary, 2e-5f,
                         std::format("Metal vs CPU freehub state ({} tick {})", label, tick));
            require_near(st_mtl[wu].drivetrain.crank_clutch.boundary,
                         st_cpu[wu].drivetrain.crank_clutch.boundary, 2e-5f,
                         std::format("Metal vs CPU crank clutch state ({} tick {})", label, tick));
        }
    }
}

void test_metal_shader_and_buffer_reuse() {
    using namespace bike::gpu;

    require(bike_metal_init(nullptr) == 0, "bike_metal_init succeeds");
    const char *dev_name = bike_metal_device_name();
    require(dev_name != nullptr && !std::string_view(dev_name).empty(),
            "bike_metal_device_name returns a non-empty device name");

    constexpr int kNWorld = 8;
    const GpuModelIndices idx{};
    std::vector<GpuWorldConfig> configs(to_index(kNWorld));
    for (int w = 0; w < kNWorld; ++w) {
        bike_gpu_init_default_config(&configs[to_index(w)]);
    }
    std::vector<GpuWorldState> states(to_index(kNWorld));

    // Reuse same host address regression
    std::vector<float> qpos(to_index(kNWorld * idx.nq), 0.0f);
    const std::span<float> qpos_rows{qpos};
    for (int w = 0; w < kNWorld; ++w) {
        const std::span<float> qpos_w = qpos_rows.subspan(to_index(w * idx.nq), to_index(idx.nq));
        qpos_w[to_index(idx.drive_shaft_spin_qpos)] = 1.0f;
        qpos_w[to_index(idx.crank_spin_qpos)] = 4.0f;
        qpos_w[to_index(idx.rear_wheel_spin_qpos)] = 0.0f;
    }
    require(bike_metal_launch_init_states(
                kNWorld, &idx, configs.data(), 1, states.data(), qpos.data()) == 0,
            "bike_metal_launch_init_states succeeds on first upload");
    const float ratio = 22.0f / 51.0f;
    for (int w = 0; w < kNWorld; ++w) {
        require_near(states[to_index(w)].drivetrain.freehub.boundary,
                     1.0f * ratio, 1e-5f, "freehub boundary from drive_shaft_spin");
        require_near(states[to_index(w)].drivetrain.crank_clutch.boundary,
                     3.0f, 1e-5f, "crank clutch boundary from crank - drive_shaft");
    }
    for (int w = 0; w < kNWorld; ++w) {
        qpos_rows.subspan(to_index(w * idx.nq), to_index(idx.nq))[to_index(idx.drive_shaft_spin_qpos)] = 10.0f;
    }
    require(bike_metal_launch_init_states(
                kNWorld, &idx, configs.data(), 1, states.data(), qpos.data()) == 0,
            "bike_metal_launch_init_states succeeds on reused address");
    for (int w = 0; w < kNWorld; ++w) {
        require_near(states[to_index(w)].drivetrain.freehub.boundary,
                     10.0f * ratio, 1e-5f, "re-initialized freehub boundary matches mutated qpos");
        require_near(states[to_index(w)].drivetrain.crank_clutch.boundary,
                     -6.0f, 1e-5f, "re-initialized crank clutch boundary matches mutated qpos");
    }

    const fs::path welded_xml = find_repo_file(kRelWeldedPath);
    require(!welded_xml.empty(), "located tests/fixtures/model/welded.xml");
    const ModelPtr m_welded = load_xml(welded_xml);
    seed_flat_welded_road(m_welded.get());
    const GpuModelIndices welded_idx = bike::gpu::assets::resolve_gpu_model_indices(m_welded.get());
    require(welded_idx.nq == 27 && welded_idx.nv == 27, "welded.xml resolves nq=27");

    verify_metal_vs_cpu_writers_parity(
        m_welded, welded_idx, make_default_compliant_tire_config(), "compliant_2d");
    verify_metal_vs_cpu_writers_parity(
        m_welded, welded_idx, make_default_distributed_tire_config(64), "distributed_2d_reference");
}

[[gnu::noinline]] void test_metal_hybrid_sweep_bounds_checks(
    const bike::gpu::GpuWorldConfig &cfg
) {
    using namespace bike::gpu;
    using namespace bike::physics_core;

    const fs::path welded_xml = find_repo_file(kRelWeldedPath);
    require(!welded_xml.empty(), "located tests/fixtures/model/welded.xml");
    const ModelPtr m_welded = load_xml(welded_xml);
    seed_flat_welded_road(m_welded.get());
    const TempWeldedAssetDir tmp(welded_xml, m_welded.get(), "mtl_bounds");
    const std::string xml_str = tmp.xml_copy.string();
    const GpuModelIndices w_idx = bike::gpu::assets::resolve_gpu_model_indices(m_welded.get());

    std::vector<double> qp0, qv0;
    DataPtr d_w{nullptr, &mj_deleteData};
    build_welded_initial_state(m_welded.get(), w_idx, qp0, qv0, d_w);
    const int f_geom = find_wheel_contact_geom(m_welded.get(), w_idx.front_wheel_body, "front_contact", "geom_front_contact");
    const int r_geom = find_wheel_contact_geom(m_welded.get(), w_idx.rear_wheel_body, "rear_contact", "geom_rear_contact");
    const auto bundle = std::make_unique<HostTyreUploadBundle>(build_gpu_tyre_model(
        m_welded.get(), d_w.get(), make_default_compliant_tire_config(),
        f_geom, r_geom, w_idx.front_wheel_body, w_idx.rear_wheel_body));
    const HostPackedTerrain w_terrain = pack_upload_bundle_terrain(*bundle);

    std::vector<float> out_fork(2, 0.0f);
    std::vector<float> out_shock(2, 0.0f);
    std::vector<float> out_rx(2, 0.0f);
    int all_fin = 0;
    double elapsed = 0.0;

    require(bike_metal_run_hybrid_sweep(
                xml_str.c_str(), 0, 2, 1, &w_idx, nullptr, nullptr, nullptr, nullptr,
                &cfg, 0, qp0.data(), qv0.data(),
                out_fork.data(), out_shock.data(), out_rx.data(), &all_fin, &elapsed) != 0,
            "metal hybrid sweep rejects nworld <= 0");
    {
        GpuModelIndices bad_idx = w_idx;
        bad_idx.nq = w_idx.nq - 1;
        require(bike_metal_run_hybrid_sweep(
                    xml_str.c_str(), 1, 2, 1, &bad_idx, nullptr, nullptr, nullptr, nullptr,
                    &cfg, 0, qp0.data(), qv0.data(),
                    out_fork.data(), out_shock.data(), out_rx.data(), &all_fin, &elapsed) != 0,
                "metal hybrid sweep rejects idx.nq != m->nq");
    }
    {
        const auto bad_tm = std::make_unique<GpuTyreModel>(bundle->gpu_model);
        const std::span bw(bad_tm->wheels);
        const std::span bj(bw[0].chain.joints);
        bj[0].joint_id = bad_tm->njnt;
        require(bike_metal_run_hybrid_sweep(
                    xml_str.c_str(), 1, 2, 1, &w_idx, bad_tm.get(), &w_terrain.desc,
                    w_terrain.f32.data(), w_terrain.i32.data(),
                    &cfg, 0, qp0.data(), qv0.data(),
                    out_fork.data(), out_shock.data(), out_rx.data(), &all_fin, &elapsed) != 0,
                "metal hybrid sweep rejects joint_id >= njnt");
    }
    {
        const auto bad_tm = std::make_unique<GpuTyreModel>(bundle->gpu_model);
        bad_tm->backend = TyreBackendKind::distributed_2d_reference;
        bad_tm->station_count = static_cast<i32>(kMaxGpuTyreStations) + 1;
        require(bike_metal_run_hybrid_sweep(
                    xml_str.c_str(), 1, 2, 1, &w_idx, bad_tm.get(), &w_terrain.desc,
                    w_terrain.f32.data(), w_terrain.i32.data(),
                    &cfg, 0, qp0.data(), qv0.data(),
                    out_fork.data(), out_shock.data(), out_rx.data(), &all_fin, &elapsed) != 0,
                "metal hybrid sweep rejects station_count > 128");
    }
    {
        auto bad_f32 = w_terrain.f32;
        bad_f32[to_index(w_terrain.desc.px_offset + 5)] =
            bad_f32[to_index(w_terrain.desc.px_offset + 4)];
        require(bike_metal_run_hybrid_sweep(
                    xml_str.c_str(), 1, 2, 1, &w_idx, &bundle->gpu_model, &w_terrain.desc,
                    bad_f32.data(), w_terrain.i32.data(),
                    &cfg, 0, qp0.data(), qv0.data(),
                    out_fork.data(), out_shock.data(), out_rx.data(), &all_fin, &elapsed) != 0,
                "metal hybrid sweep rejects non-monotonic px");
    }
}

void test_metal_launch_argument_validation() {
    using namespace bike::gpu;
    using namespace bike::physics_core;

    const GpuModelIndices idx{};
    GpuWorldConfig cfg{};
    bike_gpu_init_default_config(&cfg);
    std::vector<GpuWorldState> states(2);
    constexpr float kSentinel = 123.0f;
    for (auto &s : states) {
        s.drivetrain.freehub.boundary = kSentinel;
    }

    std::vector<float> flat_hf(64, 0.0f);
    const HostPackedTerrain terrain =
        pack_uniform_heightfield_to_terrain_tables(flat_hf, {.x_min = -5.0f, .x_max = 15.0f});
    const auto tm_ptr = std::make_unique<GpuTyreModel>();
    GpuTyreModel &tm = *tm_ptr;
    tm.backend = TyreBackendKind::compliant_2d;
    tm.nq = idx.nq;
    tm.nv = idx.nv;
    tm.nbody = idx.nbody;
    tm.njnt = 27;
    const std::span<TyreWheelSpec<float>, 2> tm_wheels{tm.wheels};
    for (int s = 0; s < 2; ++s) {
        TyreWheelSpec<float> &w = tm_wheels[to_index(s)];
        w.radius_m = 0.36f;
        w.tangent_k_n_m = 60000.0f;
        w.tire_mu = 1.1f;
        w.relaxation_length_m = 0.05f;
        w.compliant.radial_k_n_m = 110000.0f;
        w.chain.wheel_body_id = (s == 0) ? idx.front_wheel_body : idx.rear_wheel_body;
        w.chain.njnt = tm.njnt;
        w.chain.joint_count = 1;
        const std::span joints_view(w.chain.joints);
        joints_view[0] = GpuChainJoint{.joint_id = 0, .joint_type = kJntSlide, .dof_adr = 0};
    }
    tm.frame_chain.wheel_body_id = idx.frame_body;
    tm.frame_chain.njnt = tm.njnt;
    tm.frame_chain.joint_count = 1;
    std::span(tm.frame_chain.joints)[0] =
        GpuChainJoint{.joint_id = 0, .joint_type = kJntSlide, .dof_adr = 0};

    std::vector<float> buf(4096, 0.0f);
    const float *qpos = buf.data();
    const float *tf32 = terrain.f32.data();
    const i32 *ti32 = terrain.i32.data();

    require(bike_metal_launch_init_states(0, &idx, &cfg, 1, states.data(), qpos) == -1,
            "init_states rejects nworld == 0");
    require(bike_metal_launch_init_states(-3, &idx, &cfg, 1, states.data(), qpos) == -1,
            "init_states rejects nworld < 0");
    require(bike_metal_launch_init_states(2, nullptr, &cfg, 1, states.data(), qpos) == -1,
            "init_states rejects null idx");
    require(bike_metal_launch_init_states(2, &idx, nullptr, 1, states.data(), qpos) == -1,
            "init_states rejects null cfg");
    require(bike_metal_launch_init_states(2, &idx, &cfg, 1, nullptr, qpos) == -1,
            "init_states rejects null states");

    require(bike_metal_launch_writers(
                0, 0.001f, &idx, &tm, &terrain.desc, tf32, ti32, &cfg, 1, states.data(),
                qpos, qpos, qpos, qpos, qpos, qpos, buf.data(), buf.data(), buf.data(), 2) == -1,
            "writers rejects nworld == 0");
    require(bike_metal_launch_writers(
                2, 0.001f, nullptr, &tm, &terrain.desc, tf32, ti32, &cfg, 1, states.data(),
                qpos, qpos, qpos, qpos, qpos, qpos, buf.data(), buf.data(), buf.data(), 2) == -1,
            "writers rejects null idx");
    require(bike_metal_launch_writers(
                2, 0.001f, &idx, nullptr, &terrain.desc, tf32, ti32, &cfg, 1, states.data(),
                qpos, qpos, qpos, qpos, qpos, qpos, buf.data(), buf.data(), buf.data(), 2) == -1,
            "writers rejects null tyre_model");
    require(bike_metal_launch_writers(
                2, 0.001f, &idx, &tm, nullptr, tf32, ti32, &cfg, 1, states.data(),
                qpos, qpos, qpos, qpos, qpos, qpos, buf.data(), buf.data(), buf.data(), 2) == -1,
            "writers rejects null terrain_desc");

    {
        GpuModelIndices bad_idx = idx;
        bad_idx.nq = bike::gpu::kMaxNq + 1;
        require(bike_metal_launch_writers(
                    2, 0.001f, &bad_idx, &tm, &terrain.desc, tf32, ti32, &cfg, 1, states.data(),
                    qpos, qpos, qpos, qpos, qpos, qpos, buf.data(), buf.data(), buf.data(), 4) == -1,
                "writers rejects nq > kMaxNq");
        GpuModelIndices stride_idx = idx;
        stride_idx.freehub_tendon = 1;
        require(bike_metal_launch_writers(
                    2, 0.001f, &stride_idx, &tm, &terrain.desc, tf32, ti32, &cfg, 1, states.data(),
                    qpos, qpos, qpos, qpos, qpos, qpos, buf.data(), buf.data(), buf.data(), 2) == -1,
                "writers rejects small tendon_range_world_stride");
        require(bike_metal_launch_writers(
                    2, 0.001f, &idx, &tm, &terrain.desc, tf32, ti32, &cfg, 1, states.data(),
                    qpos, qpos, qpos, qpos, qpos, qpos, buf.data(), buf.data(), buf.data(), 2) == -1,
                "writers rejects stride=2 when crank_clutch_tendon=1 is active");
    }
    {
        const auto bad_tm = std::make_unique<GpuTyreModel>(tm);
        bad_tm->nq = idx.nq - 1;
        require(bike_metal_launch_writers(
                    2, 0.001f, &idx, bad_tm.get(), &terrain.desc, tf32, ti32, &cfg, 1, states.data(),
                    qpos, qpos, qpos, qpos, qpos, qpos, buf.data(), buf.data(), buf.data(), 4) == -1,
                "metal writers rejects idx.nq != tm.nq");
    }
    {
        const auto bad_tm = std::make_unique<GpuTyreModel>(tm);
        const std::span bad_wheels(bad_tm->wheels);
        const std::span bad_joints(bad_wheels[0].chain.joints);
        bad_joints[0].joint_id = bad_tm->njnt;
        require(bike_metal_launch_writers(
                    2, 0.001f, &idx, bad_tm.get(), &terrain.desc, tf32, ti32, &cfg, 1, states.data(),
                    qpos, qpos, qpos, qpos, qpos, qpos, buf.data(), buf.data(), buf.data(), 4) == -1,
                "writers rejects joint_id >= njnt");
    }
    {
        const auto bad_tm = std::make_unique<GpuTyreModel>(tm);
        bad_tm->backend = TyreBackendKind::distributed_2d_reference;
        bad_tm->station_count = static_cast<i32>(kMaxGpuTyreStations) + 1;
        require(bike_metal_launch_writers(
                    2, 0.001f, &idx, bad_tm.get(), &terrain.desc, tf32, ti32, &cfg, 1, states.data(),
                    qpos, qpos, qpos, qpos, qpos, qpos, buf.data(), buf.data(), buf.data(), 4) == -1,
                "writers rejects station_count > 128");
    }
    {
        auto bad_f32 = terrain.f32;
        bad_f32[to_index(terrain.desc.px_offset + 5)] =
            bad_f32[to_index(terrain.desc.px_offset + 4)];
        require(bike_metal_launch_writers(
                    2, 0.001f, &idx, &tm, &terrain.desc, bad_f32.data(), ti32, &cfg, 1, states.data(),
                    qpos, qpos, qpos, qpos, qpos, qpos, buf.data(), buf.data(), buf.data(), 4) == -1,
                "writers rejects non-monotonic px");
    }
    for (const auto &s : states) {
        require(s.drivetrain.freehub.boundary == kSentinel,
                "rejected writers leaves state buffer untouched");
    }

    // bike_metal_run_hybrid_sweep negative bounds checks on welded.xml
    test_metal_hybrid_sweep_bounds_checks(cfg);
}
void test_metal_single_world_stride0_dual_tendon() {
    using namespace bike::gpu;
    using namespace bike::physics_core;

    require(bike_metal_init(nullptr) == 0, "bike_metal_init succeeds");
    const fs::path welded_xml = find_repo_file(kRelWeldedPath);
    require(!welded_xml.empty(), "located tests/fixtures/model/welded.xml");
    const ModelPtr m_welded = load_xml(welded_xml);
    seed_flat_welded_road(m_welded.get());
    const GpuModelIndices idx = bike::gpu::assets::resolve_gpu_model_indices(m_welded.get());
    require(idx.nq == 27 && idx.freehub_tendon == 0 && idx.crank_clutch_tendon == 1 &&
                idx.drive_shaft_spin_qpos >= 0,
            "welded.xml has dual tendons freehub=0 and crank_clutch=1");

    DataPtr d(mj_makeData(m_welded.get()), &mj_deleteData);
    mj_resetData(m_welded.get(), d.get());
    const std::span<mjtNum> d_qpos = model_access::mutable_buffer(d->qpos, idx.nq, "qpos");
    const std::span<mjtNum> d_qvel = model_access::mutable_buffer(d->qvel, idx.nv, "qvel");
    d_qpos[to_index(idx.root_x_qpos)] = 2.0;
    d_qpos[to_index(idx.root_z_qpos)] = 0.50;
    d_qpos[to_index(idx.crank_spin_qpos)] = 3.5;
    d_qpos[to_index(idx.drive_shaft_spin_qpos)] = 1.2;
    d_qpos[to_index(idx.rear_wheel_spin_qpos)] = 0.4;
    d_qvel[to_index(idx.root_x_dof)] = 5.0;
    d_qvel[to_index(idx.crank_spin_dof)] = 8.0;
    mj_forward(m_welded.get(), d.get());

    const int f_geom = find_wheel_contact_geom(m_welded.get(), idx.front_wheel_body, "front_contact", "geom_front_contact");
    const int r_geom = find_wheel_contact_geom(m_welded.get(), idx.rear_wheel_body, "rear_contact", "geom_rear_contact");
    const auto bundle = std::make_unique<HostTyreUploadBundle>(build_gpu_tyre_model(
        m_welded.get(), d.get(), make_default_compliant_tire_config(),
        f_geom, r_geom, idx.front_wheel_body, idx.rear_wheel_body));
    const HostPackedTerrain terrain = pack_upload_bundle_terrain(*bundle);

    GpuWorldConfig cfg{};
    bike_gpu_init_default_config(&cfg);
    const int nq = idx.nq;
    const int nv = idx.nv;
    const int nu = idx.nu;
    const int nbody = idx.nbody;
    const int njnt = static_cast<int>(m_welded->njnt);

    std::vector<float> qpos_w(to_index(nq), 0.0f);
    std::vector<float> qvel_w(to_index(nv), 0.0f);
    for (int i = 0; i < nq; ++i) qpos_w[to_index(i)] = static_cast<float>(d_qpos[to_index(i)]);
    for (int i = 0; i < nv; ++i) qvel_w[to_index(i)] = static_cast<float>(d_qvel[to_index(i)]);

    const std::span<const mjtNum> src_xpos = model_access::readonly_buffer(d->xpos, 3LL * static_cast<mjtSize>(nbody));
    const std::span<const mjtNum> src_xmat = model_access::readonly_buffer(d->xmat, 9LL * static_cast<mjtSize>(nbody));
    const std::span<const mjtNum> src_xanc = model_access::readonly_buffer(d->xanchor, 3LL * static_cast<mjtSize>(njnt));
    const std::span<const mjtNum> src_xax = model_access::readonly_buffer(d->xaxis, 3LL * static_cast<mjtSize>(njnt));
    std::vector<float> xpos_w(src_xpos.size());
    std::vector<float> xmat_w(src_xmat.size());
    std::vector<float> xanchor_w(src_xanc.size());
    std::vector<float> xaxis_w(src_xax.size());
    for (std::size_t i = 0; i < src_xpos.size(); ++i) xpos_w[i] = static_cast<float>(src_xpos[i]);
    for (std::size_t i = 0; i < src_xmat.size(); ++i) xmat_w[i] = static_cast<float>(src_xmat[i]);
    for (std::size_t i = 0; i < src_xanc.size(); ++i) xanchor_w[i] = static_cast<float>(src_xanc[i]);
    for (std::size_t i = 0; i < src_xax.size(); ++i) xaxis_w[i] = static_cast<float>(src_xax[i]);

    const auto st_mtl = std::make_unique<GpuWorldState>();
    const auto st_cpu = std::make_unique<GpuWorldState>();
    require(bike_metal_launch_init_states(1, &idx, &cfg, 1, st_mtl.get(), qpos_w.data()) == 0,
            "stride0 metal init_states ok");
    require(bike_gpu_launch_init_states(nullptr, 1, &idx, &cfg, 1, st_cpu.get(), qpos_w.data()) == 0,
            "stride0 cpu init_states ok");

    std::vector<float> qfrc_mtl(to_index(nv), 0.0f);
    std::vector<float> qfrc_cpu(to_index(nv), 0.0f);
    std::vector<float> ctrl_mtl(to_index(nu), 0.0f);
    std::vector<float> ctrl_cpu(to_index(nu), 0.0f);
    // 4 active floats + 2 trailing canary floats to catch any OOB write on stride == 0
    std::array<float, 6> tendon_mtl{-100.0f, 99.0f, -200.0f, 99.0f, 777.0f, 888.0f};
    std::array<float, 6> tendon_cpu{-100.0f, 99.0f, -200.0f, 99.0f, 777.0f, 888.0f};

    const float *tf32 = terrain.f32.empty() ? nullptr : terrain.f32.data();
    const i32 *ti32 = terrain.i32.empty() ? nullptr : terrain.i32.data();
    for (int tick = 0; tick < 3; ++tick) {
        require(bike_metal_launch_writers(
                    1, 0.001f, &idx, &bundle->gpu_model, &terrain.desc, tf32, ti32,
                    &cfg, 1, st_mtl.get(),
                    qpos_w.data(), qvel_w.data(),
                    xpos_w.data(), xmat_w.data(), xanchor_w.data(), xaxis_w.data(),
                    qfrc_mtl.data(), ctrl_mtl.data(), tendon_mtl.data(), 0) == 0,
                "stride0 metal launch_writers ok");
        require(bike_gpu_launch_writers(
                    nullptr, 1, 0.001f, &idx, &bundle->gpu_model, &terrain.desc, tf32, ti32,
                    &cfg, 1, st_cpu.get(),
                    qpos_w.data(), qvel_w.data(),
                    xpos_w.data(), xmat_w.data(), xanchor_w.data(), xaxis_w.data(),
                    qfrc_cpu.data(), ctrl_cpu.data(), tendon_cpu.data(), 0) == 0,
                "stride0 cpu launch_writers ok");
        for (std::size_t i = 0; i < 4; ++i) {
            require_near(tendon_mtl[i], tendon_cpu[i], 2e-5f,
                         std::format("stride0 Metal vs CPU tendon_range[{}] (tick {})", i, tick));
        }
        require(tendon_mtl[1] != 99.0f && tendon_mtl[3] != 99.0f,
                "stride0 updates both freehub and crank_clutch upper bounds");
        require(tendon_mtl[4] == 777.0f && tendon_mtl[5] == 888.0f,
                "stride0 does not overwrite past 4 tendon_range floats");
    }
}
#endif // defined(__APPLE__)

void verify_hybrid_rollout_backend(
    const std::string &xml_str,
    const bike::gpu::GpuModelIndices &idx,
    const bike::physics_core::GpuTyreModel *tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *terrain_desc,
    const float *terrain_f32,
    const bike::physics_core::i32 *terrain_i32,
    int nworld,
    int n_steps,
    const std::vector<bike::gpu::GpuWorldConfig> &configs,
    std::string_view label
) {
    std::vector<float> cpu_fork_mm(to_index(nworld), 0.0f);
    std::vector<float> cpu_shock_mm(to_index(nworld), 0.0f);
    std::vector<float> cpu_root_x_m(to_index(nworld), 0.0f);
    int cpu_finite = 0;
    double cpu_ms = 0.0;

    const int rc_cpu = bike_cpu_run_batch_sweep(
        xml_str.c_str(), nworld, n_steps, 2, &idx,
        tyre_model, terrain_desc, terrain_f32, terrain_i32,
        configs.data(), 1, nullptr, nullptr,
        cpu_fork_mm.data(), cpu_shock_mm.data(), cpu_root_x_m.data(),
        &cpu_finite, &cpu_ms
    );
    require(rc_cpu == 0, std::format("bike_cpu_run_batch_sweep returns 0 ({})", label));
    require(cpu_finite == 1, std::format("CPU rollout stays finite ({})", label));

    constexpr double kX0M = 2.0163204663006566;
    constexpr double kMinAdvanceM = 0.01;
    constexpr double kMaxAdvanceM = 0.50;
    for (int w = 0; w < nworld; ++w) {
        const std::size_t wi = to_index(w);
        require_above(static_cast<double>(cpu_root_x_m[wi]), kX0M + kMinAdvanceM,
                      "cpu root_x advances past captured start");
        require_below(static_cast<double>(cpu_root_x_m[wi]), kX0M + kMaxAdvanceM,
                      "cpu root_x stays below max speed bound");
        require(cpu_fork_mm[wi] > 20.0f && cpu_fork_mm[wi] < 130.0f,
                "cpu peak fork travel inside physical band");
        require(cpu_shock_mm[wi] > 5.0f && cpu_shock_mm[wi] < 60.0f,
                "cpu peak shock stroke inside physical band");
    }

#if defined(__APPLE__)
    std::vector<float> mtl_fork_mm(to_index(nworld), 0.0f);
    std::vector<float> mtl_shock_mm(to_index(nworld), 0.0f);
    std::vector<float> mtl_root_x_m(to_index(nworld), 0.0f);
    int mtl_finite = 0;
    double mtl_ms = 0.0;

    const int rc_mtl = bike_metal_run_hybrid_sweep(
        xml_str.c_str(), nworld, n_steps, 2, &idx,
        tyre_model, terrain_desc, terrain_f32, terrain_i32,
        configs.data(), 1, nullptr, nullptr,
        mtl_fork_mm.data(), mtl_shock_mm.data(), mtl_root_x_m.data(),
        &mtl_finite, &mtl_ms
    );
    require(rc_mtl == 0, std::format("bike_metal_run_hybrid_sweep returns 0 ({})", label));
    require(mtl_finite == 1, std::format("Metal rollout stays finite ({})", label));
    for (int w = 0; w < nworld; ++w) {
        const std::size_t wi = to_index(w);
        require_near(mtl_root_x_m[wi], cpu_root_x_m[wi],
                     3e-3f * std::max(1.0f, std::abs(cpu_root_x_m[wi])),
                     std::format("metal vs cpu root_x ({})", label));
        require_near(mtl_fork_mm[wi], cpu_fork_mm[wi],
                     3e-3f * std::max(1.0f, std::abs(cpu_fork_mm[wi])),
                     std::format("metal vs cpu fork_mm ({})", label));
        require_near(mtl_shock_mm[wi], cpu_shock_mm[wi],
                     3e-3f * std::max(1.0f, std::abs(cpu_shock_mm[wi])),
                     std::format("metal vs cpu shock_mm ({})", label));
    }
#endif
}

void test_cpu_and_metal_hybrid_rollouts() {
    using namespace bike::gpu;
    using namespace bike::physics_core;

    const fs::path xml_path = find_repo_file(kRelModelPath);
    if (xml_path.empty()) {
        std::cout << "SKIP model.xml rollout check (" << kRelModelPath << " not found)\n";
        return;
    }
    const std::string xml_str = xml_path.string();
    constexpr int kNWorld = 4;
    constexpr int kSteps = 400;

    const ModelPtr m_probe = load_xml(xml_path);
    require(assets::load_hfield_bin(m_probe.get(), xml_str) == assets::AssetStatus::kOk,
            "load_hfield_bin succeeds for probe model.xml");
    const GpuModelIndices idx = assets::resolve_gpu_model_indices(m_probe.get());
    require(idx.nq == 26 && idx.drive_shaft_spin_qpos == -1 && idx.crank_clutch_tendon == -1,
            "resolve_gpu_model_indices resolves nq=26 probe model");

    std::vector<GpuWorldConfig> configs(to_index(kNWorld));
    for (int w = 0; w < kNWorld; ++w) {
        bike_gpu_init_default_config(&configs[to_index(w)]);
    }

    // 1. Default compliant_2d rollout parity (null tyre/terrain overrides)
    verify_hybrid_rollout_backend(
        xml_str, idx, nullptr, nullptr, nullptr, nullptr,
        kNWorld, kSteps, configs, "compliant_default");

    // 2. Explicit distributed_2d_reference (64 stations) rollout parity
    DataPtr d_probe(mj_makeData(m_probe.get()), &mj_deleteData);
    std::vector<double> qpos0, qvel0;
    require(assets::load_captured_initial_state(xml_str, idx.nq, idx.nv, qpos0, qvel0) ==
                assets::AssetStatus::kOk,
            "captured_bootstrap.json loads");
    const std::span<mjtNum> qp0 = model_access::mutable_buffer(d_probe->qpos, idx.nq);
    const std::span<mjtNum> qv0 = model_access::mutable_buffer(d_probe->qvel, idx.nv);
    for (int i = 0; i < idx.nq; ++i) qp0[to_index(i)] = qpos0[to_index(i)];
    for (int i = 0; i < idx.nv; ++i) qv0[to_index(i)] = qvel0[to_index(i)];
    mj_forward(m_probe.get(), d_probe.get());

    const int f_geom = find_wheel_contact_geom(
        m_probe.get(), idx.front_wheel_body, "front_contact", "geom_front_contact");
    const int r_geom = find_wheel_contact_geom(
        m_probe.get(), idx.rear_wheel_body, "rear_contact", "geom_rear_contact");
    require(f_geom >= 0 && r_geom >= 0, "found front/rear contact geoms in model.xml");
    const auto dist_bundle = std::make_unique<HostTyreUploadBundle>(build_gpu_tyre_model(
        m_probe.get(), d_probe.get(), make_default_distributed_tire_config(64),
        f_geom, r_geom, idx.front_wheel_body, idx.rear_wheel_body));
    const HostPackedTerrain dist_terrain = pack_upload_bundle_terrain(*dist_bundle);

    verify_hybrid_rollout_backend(
        xml_str, idx, &dist_bundle->gpu_model, &dist_terrain.desc,
        dist_terrain.f32.data(), dist_terrain.i32.empty() ? nullptr : dist_terrain.i32.data(),
        kNWorld, kSteps, configs, "distributed_2d_reference");

    // 3. Explicit custom terrain override (low vs elevated) shifts fork travel peak
    const auto comp_bundle = std::make_unique<HostTyreUploadBundle>(build_gpu_tyre_model(
        m_probe.get(), d_probe.get(), make_default_compliant_tire_config(),
        f_geom, r_geom, idx.front_wheel_body, idx.rear_wheel_body));
    const std::vector<float> z_low(2000, -0.390f);
    const std::vector<float> z_high(2000, -0.320f);
    const HostPackedTerrain t_low =
        pack_uniform_heightfield_to_terrain_tables(z_low, {.x_min = -2.0f, .x_max = 17.99f});
    const HostPackedTerrain t_high =
        pack_uniform_heightfield_to_terrain_tables(z_high, {.x_min = -2.0f, .x_max = 17.99f});

    std::vector<float> fork_low(to_index(kNWorld), 0.0f);
    std::vector<float> fork_high(to_index(kNWorld), 0.0f);
    std::vector<float> tmp_shock(to_index(kNWorld), 0.0f);
    std::vector<float> tmp_rx(to_index(kNWorld), 0.0f);
    int fin_low = 0, fin_high = 0;
    double ms_tmp = 0.0;
    require(bike_cpu_run_batch_sweep(
                xml_str.c_str(), kNWorld, 50, 2, &idx,
                &comp_bundle->gpu_model, &t_low.desc, t_low.f32.data(), t_low.i32.data(),
                configs.data(), 1, nullptr, nullptr,
                fork_low.data(), tmp_shock.data(), tmp_rx.data(), &fin_low, &ms_tmp) == 0,
            "custom terrain (low) succeeds");
    require(bike_cpu_run_batch_sweep(
                xml_str.c_str(), kNWorld, 50, 2, &idx,
                &comp_bundle->gpu_model, &t_high.desc, t_high.f32.data(), t_high.i32.data(),
                configs.data(), 1, nullptr, nullptr,
                fork_high.data(), tmp_shock.data(), tmp_rx.data(), &fin_high, &ms_tmp) == 0,
            "custom terrain (high) succeeds");
    require(fin_low == 1 && fin_high == 1, "custom terrain sweeps stay finite");
    require(fork_high[0] > fork_low[0] + 1.0f,
            "higher custom terrain table raises peak fork compression");
}

void test_cpu_and_host_argument_validation() {
    using namespace bike::gpu;
    using namespace bike::physics_core;

    const fs::path welded_xml = find_repo_file(kRelWeldedPath);
    require(!welded_xml.empty(), "located tests/fixtures/model/welded.xml");
    const ModelPtr m_welded = load_xml(welded_xml);
    seed_flat_welded_road(m_welded.get());
    const TempWeldedAssetDir tmp(welded_xml, m_welded.get(), "cpu_bounds");
    const std::string xml_str = tmp.xml_copy.string();
    const GpuModelIndices idx = bike::gpu::assets::resolve_gpu_model_indices(m_welded.get());
    require(idx.nq == 27 && idx.nv == 27, "welded.xml resolves nq=27");

    std::vector<double> qp0, qv0;
    DataPtr d_w{nullptr, &mj_deleteData};
    build_welded_initial_state(m_welded.get(), idx, qp0, qv0, d_w);
    const int f_geom = find_wheel_contact_geom(m_welded.get(), idx.front_wheel_body, "front_contact", "geom_front_contact");
    const int r_geom = find_wheel_contact_geom(m_welded.get(), idx.rear_wheel_body, "rear_contact", "geom_rear_contact");
    const auto bundle = std::make_unique<HostTyreUploadBundle>(build_gpu_tyre_model(
        m_welded.get(), d_w.get(), make_default_compliant_tire_config(),
        f_geom, r_geom, idx.front_wheel_body, idx.rear_wheel_body));
    const HostPackedTerrain terrain = pack_upload_bundle_terrain(*bundle);

    GpuWorldConfig cfg{};
    bike_gpu_init_default_config(&cfg);
    std::vector<GpuWorldState> states(2);
    constexpr float kSentinel = 321.0f;
    for (auto &s : states) {
        s.drivetrain.freehub.boundary = kSentinel;
    }
    std::vector<float> buf(4096, 0.0f);
    const float *qpos = buf.data();
    const float *tf32 = terrain.f32.data();
    const i32 *ti32 = terrain.i32.data();

    // 1. Host writers launcher (bike_gpu_launch_writers) negative bounds
    require(bike_gpu_launch_writers(
                nullptr, 0, 0.001f, &idx, &bundle->gpu_model, &terrain.desc, tf32, ti32,
                &cfg, 1, states.data(), qpos, qpos, qpos, qpos, qpos, qpos,
                buf.data(), buf.data(), buf.data(), 4) != 0,
            "host writers rejects nworld <= 0");
    require(bike_gpu_launch_writers(
                nullptr, 2, 0.001f, &idx, &bundle->gpu_model, &terrain.desc, tf32, ti32,
                &cfg, 1, states.data(), qpos, qpos, qpos, qpos, qpos, qpos,
                buf.data(), buf.data(), buf.data(), 2) != 0,
            "host writers rejects stride=2 when crank_clutch_tendon=1 is active");
    {
        GpuModelIndices bad_idx = idx;
        bad_idx.nq = idx.nq - 1;
        require(bike_gpu_launch_writers(
                    nullptr, 2, 0.001f, &bad_idx, &bundle->gpu_model, &terrain.desc, tf32, ti32,
                    &cfg, 1, states.data(), qpos, qpos, qpos, qpos, qpos, qpos,
                    buf.data(), buf.data(), buf.data(), 4) != 0,
                "host writers rejects idx.nq != tm.nq");
    }
    {
        const auto bad_tm = std::make_unique<GpuTyreModel>(bundle->gpu_model);
        const std::span bw(bad_tm->wheels);
        const std::span bj(bw[0].chain.joints);
        bj[0].joint_id = bad_tm->njnt;
        require(bike_gpu_launch_writers(
                    nullptr, 2, 0.001f, &idx, bad_tm.get(), &terrain.desc, tf32, ti32,
                    &cfg, 1, states.data(), qpos, qpos, qpos, qpos, qpos, qpos,
                    buf.data(), buf.data(), buf.data(), 4) != 0,
                "host writers rejects joint_id >= njnt");
    }
    {
        const auto bad_tm = std::make_unique<GpuTyreModel>(bundle->gpu_model);
        bad_tm->backend = TyreBackendKind::distributed_2d_reference;
        bad_tm->station_count = static_cast<i32>(kMaxGpuTyreStations) + 1;
        require(bike_gpu_launch_writers(
                    nullptr, 2, 0.001f, &idx, bad_tm.get(), &terrain.desc, tf32, ti32,
                    &cfg, 1, states.data(), qpos, qpos, qpos, qpos, qpos, qpos,
                    buf.data(), buf.data(), buf.data(), 4) != 0,
                "host writers rejects station_count > 128");
    }
    {
        auto bad_f32 = terrain.f32;
        bad_f32[to_index(terrain.desc.px_offset + 5)] =
            bad_f32[to_index(terrain.desc.px_offset + 4)];
        require(bike_gpu_launch_writers(
                    nullptr, 2, 0.001f, &idx, &bundle->gpu_model, &terrain.desc, bad_f32.data(), ti32,
                    &cfg, 1, states.data(), qpos, qpos, qpos, qpos, qpos, qpos,
                    buf.data(), buf.data(), buf.data(), 4) != 0,
                "host writers rejects non-monotonic px");
    }
    for (const auto &s : states) {
        require(s.drivetrain.freehub.boundary == kSentinel,
                "rejected host writers leaves state buffer untouched");
    }

    // 2. CPU batch sweep (bike_cpu_run_batch_sweep) negative bounds
    std::vector<float> out_fork(2, 0.0f);
    std::vector<float> out_shock(2, 0.0f);
    std::vector<float> out_rx(2, 0.0f);
    int all_fin = 0;
    double elapsed = 0.0;

    require(bike_cpu_run_batch_sweep(
                xml_str.c_str(), 0, 2, 1, &idx, nullptr, nullptr, nullptr, nullptr,
                &cfg, 0, qp0.data(), qv0.data(),
                out_fork.data(), out_shock.data(), out_rx.data(), &all_fin, &elapsed) != 0,
            "cpu batch sweep rejects nworld <= 0");
    {
        GpuModelIndices bad_idx = idx;
        bad_idx.nq = idx.nq - 1;
        require(bike_cpu_run_batch_sweep(
                    xml_str.c_str(), 1, 2, 1, &bad_idx, nullptr, nullptr, nullptr, nullptr,
                    &cfg, 0, qp0.data(), qv0.data(),
                    out_fork.data(), out_shock.data(), out_rx.data(), &all_fin, &elapsed) != 0,
                "cpu batch sweep rejects idx.nq != m->nq");
    }
    {
        const auto bad_tm = std::make_unique<GpuTyreModel>(bundle->gpu_model);
        const std::span bw(bad_tm->wheels);
        const std::span bj(bw[0].chain.joints);
        bj[0].joint_id = bad_tm->njnt;
        require(bike_cpu_run_batch_sweep(
                    xml_str.c_str(), 1, 2, 1, &idx, bad_tm.get(), &terrain.desc,
                    tf32, ti32, &cfg, 0, qp0.data(), qv0.data(),
                    out_fork.data(), out_shock.data(), out_rx.data(), &all_fin, &elapsed) != 0,
                "cpu batch sweep rejects joint_id >= njnt");
    }
    {
        const auto bad_tm = std::make_unique<GpuTyreModel>(bundle->gpu_model);
        bad_tm->backend = TyreBackendKind::distributed_2d_reference;
        bad_tm->station_count = static_cast<i32>(kMaxGpuTyreStations) + 1;
        require(bike_cpu_run_batch_sweep(
                    xml_str.c_str(), 1, 2, 1, &idx, bad_tm.get(), &terrain.desc,
                    tf32, ti32, &cfg, 0, qp0.data(), qv0.data(),
                    out_fork.data(), out_shock.data(), out_rx.data(), &all_fin, &elapsed) != 0,
                "cpu batch sweep rejects station_count > 128");
    }
    {
        auto bad_f32 = terrain.f32;
        bad_f32[to_index(terrain.desc.px_offset + 5)] =
            bad_f32[to_index(terrain.desc.px_offset + 4)];
        require(bike_cpu_run_batch_sweep(
                    xml_str.c_str(), 1, 2, 1, &idx, &bundle->gpu_model, &terrain.desc,
                    bad_f32.data(), ti32, &cfg, 0, qp0.data(), qv0.data(),
                    out_fork.data(), out_shock.data(), out_rx.data(), &all_fin, &elapsed) != 0,
                "cpu batch sweep rejects non-monotonic px");
    }
}

void test_welded_80step_rollout_parity() {
    using namespace bike::gpu;
    using namespace bike::physics_core;

    const fs::path welded_xml = find_repo_file(kRelWeldedPath);
    require(!welded_xml.empty(), "located tests/fixtures/model/welded.xml");
    const ModelPtr m_welded = load_xml(welded_xml);
    require(m_welded->nq == 27 && m_welded->nv == 27 && m_welded->nu == 12 &&
                m_welded->ntendon == 2,
            "welded.xml has nq=27, nv=27, nu=12, ntendon=2");
    seed_flat_welded_road(m_welded.get());
    const TempWeldedAssetDir tmp(welded_xml, m_welded.get(), "welded_80");
    const std::string xml_str = tmp.xml_copy.string();
    const GpuModelIndices idx = bike::gpu::assets::resolve_gpu_model_indices(m_welded.get());

    std::vector<double> qp0, qv0;
    DataPtr d_w{nullptr, &mj_deleteData};
    build_welded_initial_state(m_welded.get(), idx, qp0, qv0, d_w);
    const int f_geom = find_wheel_contact_geom(m_welded.get(), idx.front_wheel_body, "front_contact", "geom_front_contact");
    const int r_geom = find_wheel_contact_geom(m_welded.get(), idx.rear_wheel_body, "rear_contact", "geom_rear_contact");

    constexpr int kNWorld = 4;
    constexpr int kSteps = 80;
    std::vector<GpuWorldConfig> configs(to_index(kNWorld));
    for (int w = 0; w < kNWorld; ++w) {
        bike_gpu_init_default_config(&configs[to_index(w)]);
    }

    const auto run_welded_backend = [&](const nativecfg::TireConfig &tcfg, std::string_view label) {
        const auto bundle = std::make_unique<HostTyreUploadBundle>(build_gpu_tyre_model(
            m_welded.get(), d_w.get(), tcfg,
            f_geom, r_geom, idx.front_wheel_body, idx.rear_wheel_body));
        const HostPackedTerrain terrain = pack_upload_bundle_terrain(*bundle);

        std::vector<float> cpu_fork(to_index(kNWorld), 0.0f);
        std::vector<float> cpu_shock(to_index(kNWorld), 0.0f);
        std::vector<float> cpu_rx(to_index(kNWorld), 0.0f);
        int cpu_finite = 0;
        double cpu_ms = 0.0;
        const int rc_cpu = bike_cpu_run_batch_sweep(
            xml_str.c_str(), kNWorld, kSteps, 2, &idx,
            &bundle->gpu_model, &terrain.desc,
            terrain.f32.empty() ? nullptr : terrain.f32.data(),
            terrain.i32.empty() ? nullptr : terrain.i32.data(),
            configs.data(), 1, qp0.data(), qv0.data(),
            cpu_fork.data(), cpu_shock.data(), cpu_rx.data(),
            &cpu_finite, &cpu_ms);
        require(rc_cpu == 0, std::format("welded CPU 80-step sweep succeeds ({}, rc={})", label, rc_cpu));
        require(cpu_finite == 1, std::format("welded CPU 80-step stays finite ({})", label));
        for (int w = 0; w < kNWorld; ++w) {
            require_above(static_cast<double>(cpu_rx[to_index(w)]), 2.0 + 0.1,
                          std::format("welded CPU root_x advances ({})", label));
        }

#if defined(__APPLE__)
        std::vector<float> mtl_fork(to_index(kNWorld), 0.0f);
        std::vector<float> mtl_shock(to_index(kNWorld), 0.0f);
        std::vector<float> mtl_rx(to_index(kNWorld), 0.0f);
        int mtl_finite = 0;
        double mtl_ms = 0.0;
        const int rc_mtl = bike_metal_run_hybrid_sweep(
            xml_str.c_str(), kNWorld, kSteps, 2, &idx,
            &bundle->gpu_model, &terrain.desc,
            terrain.f32.empty() ? nullptr : terrain.f32.data(),
            terrain.i32.empty() ? nullptr : terrain.i32.data(),
            configs.data(), 1, qp0.data(), qv0.data(),
            mtl_fork.data(), mtl_shock.data(), mtl_rx.data(),
            &mtl_finite, &mtl_ms);
        require(rc_mtl == 0, std::format("welded Metal hybrid 80-step sweep succeeds ({})", label));
        require(mtl_finite == 1, std::format("welded Metal hybrid 80-step stays finite ({})", label));
        for (int w = 0; w < kNWorld; ++w) {
            const std::size_t wi = to_index(w);
            require_near(mtl_rx[wi], cpu_rx[wi],
                         3e-3f * std::max(1.0f, std::abs(cpu_rx[wi])),
                         std::format("welded metal vs cpu root_x ({})", label));
            require_near(mtl_fork[wi], cpu_fork[wi],
                         3e-3f * std::max(1.0f, std::abs(cpu_fork[wi])),
                         std::format("welded metal vs cpu fork_mm ({})", label));
            require_near(mtl_shock[wi], cpu_shock[wi],
                         3e-3f * std::max(1.0f, std::abs(cpu_shock[wi])),
                         std::format("welded metal vs cpu shock_mm ({})", label));
        }
#endif
    };

    run_welded_backend(make_default_compliant_tire_config(), "compliant_2d");
    run_welded_backend(make_default_distributed_tire_config(64), "distributed_2d_reference");
}

#if defined(__APPLE__)
constexpr std::array<TestCase, 6> cases{{
    {.name = "metal_shader_and_buffer_reuse",
     .run = test_metal_shader_and_buffer_reuse},
    {.name = "metal_launch_argument_validation",
     .run = test_metal_launch_argument_validation},
    {.name = "metal_single_world_stride0_dual_tendon",
     .run = test_metal_single_world_stride0_dual_tendon},
    {.name = "cpu_and_host_argument_validation",
     .run = test_cpu_and_host_argument_validation},
    {.name = "cpu_and_metal_hybrid_rollouts",
     .run = test_cpu_and_metal_hybrid_rollouts},
    {.name = "welded_80step_rollout_parity",
     .run = test_welded_80step_rollout_parity},
}};
#else
constexpr std::array<TestCase, 3> cases{{
    {.name = "cpu_and_host_argument_validation",
     .run = test_cpu_and_host_argument_validation},
    {.name = "cpu_and_metal_hybrid_rollouts",
     .run = test_cpu_and_metal_hybrid_rollouts},
    {.name = "welded_80step_rollout_parity",
     .run = test_welded_80step_rollout_parity},
}};
#endif

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
            for (const TestCase &tc : cases) {
                std::cout << tc.name << '\n';
            }
            return 0;
        }
        if (arguments.size() > 2) {
            std::cerr << "usage: bike_test_gpu_batch_runners [case-name|--list]\n";
            return 2;
        }
        const std::string_view requested =
            arguments.size() == 1 ? "all" : std::string_view(arguments[1]);
        bool found = requested == "all";
        int status = 0;
        for (const TestCase &tc : cases) {
            if (requested == "all" || requested == tc.name) {
                found = true;
                if (run_case(tc) != 0) {
                    status = 1;
                }
            }
        }
        if (!found) {
            std::cerr << "unknown gpu batch_runners case: " << requested << '\n';
            return 2;
        }
        return status;
    } catch (const std::exception &error) {
        std::cerr << "FAIL argument handling: " << error.what() << '\n';
        return 1;
    }
}
