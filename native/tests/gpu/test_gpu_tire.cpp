// native/tests/gpu/test_gpu_tire.cpp — C++23 parity & contract verification for
// native/src/physics_core/tyre_gpu_step.hpp (compliant_2d + distributed_2d_reference)
// against CPU FP64 TireWriter & DistributedTireWriter on welded.xml (nq=27).

#include "../../src/diag.hpp"
#include "../../src/gpu/batch_step.hpp"
#include "../../src/gpu/model_assets.hpp"
#include "../../src/gpu/runner_tyre_setup.hpp"
#include "../../src/gpu/terrain_tables.hpp"
#include "../../src/model_access.hpp"
#include "../../src/physics_core/tyre_gpu_step.hpp"
#include "../../src/physics_core/tyre_upload.hpp"
#include "../../src/platform/argv.hpp"
#include "../../src/writers/distributed_tire.hpp"
#include "../../src/writers/pneumatic_tire.hpp"
#include "../../src/writers/tire.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <format>
#include <iostream>
#include <limits>
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
#endif
} // extern "C"

namespace {

namespace fs = std::filesystem;
using namespace bike::physics_core;
using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPtr = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

using TestFunction = void (*)(const fs::path &);

struct TestCase {
    std::string_view name;
    TestFunction run;
};

void require(bool condition, std::string_view message) {
    if (!condition) {
        throw std::runtime_error(std::string(message));
    }
}

constexpr std::size_t to_index(int idx) {
    return static_cast<std::size_t>(idx);
}

constexpr const char *kRelWeldedXml = "tests/fixtures/model/welded.xml";

fs::path resolve_welded_xml(std::string_view arg) {
    if (!arg.empty()) {
        return {arg};
    }
    std::vector<fs::path> candidates;
#ifdef BIKE_REPO_ROOT
    candidates.emplace_back(fs::path(BIKE_REPO_ROOT) / kRelWeldedXml);
#endif
    candidates.emplace_back(kRelWeldedXml);
    candidates.emplace_back(fs::path("..") / kRelWeldedXml);
    candidates.emplace_back(fs::path("../..") / kRelWeldedXml);
    candidates.emplace_back(fs::path("../../..") / kRelWeldedXml);
    for (const fs::path &p : candidates) {
        std::error_code ec;
        if (fs::is_regular_file(p, ec)) {
            return fs::canonical(p, ec);
        }
    }
    return {};
}

double ground_profile_z(double x, bool rough_uphill) {
    if (!rough_uphill) {
        return 0.0;
    }
    const double slope = (x > 2.0) ? 0.15 * (x - 2.0) : 0.0;
    return slope + 0.015 * std::sin(7.0 * x) + 0.008 * std::cos(13.0 * x);
}

void populate_welded_terrain(const mjModel *m, bool rough_uphill) {
    const int hf_id = mj_name2id(m, mjOBJ_HFIELD, "road");
    require(hf_id >= 0, "welded.xml must contain 'road' hfield");
    const int terrain_geom = mj_name2id(m, mjOBJ_GEOM, "terrain");
    require(terrain_geom >= 0, "welded.xml must contain 'terrain' geom");

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
    const std::span<float> hfield_data =
        model_access::mutable_buffer(m->hfield_data, m->nhfielddata, "hfield_data");

    const int nrow = hfield_nrow[to_index(hf_id)];
    const int ncol = hfield_ncol[to_index(hf_id)];
    const int adr = hfield_adr[to_index(hf_id)];
    const double sx = static_cast<double>(hfield_size[4 * to_index(hf_id) + 0]);
    const double sz = static_cast<double>(hfield_size[4 * to_index(hf_id) + 2]);
    const double pos_x = static_cast<double>(geom_pos[3 * to_index(terrain_geom) + 0]);
    const double pos_z = static_cast<double>(geom_pos[3 * to_index(terrain_geom) + 2]);
    const double dx = (2.0 * sx) / static_cast<double>(ncol - 1);

    for (int c = 0; c < ncol; ++c) {
        const double lx = (c + 1 == ncol) ? sx : (static_cast<double>(c) * dx - sx);
        const double wx = pos_x + lx;
        const double wz = ground_profile_z(wx, rough_uphill);
        const float norm_h = static_cast<float>((wz - pos_z) / sz);
        for (int r = 0; r < nrow; ++r) {
            hfield_data[to_index(adr + r * ncol + c)] = norm_h;
        }
    }
}

struct WeldedFixture {
    ModelPtr m{nullptr, &mj_deleteModel};
    DataPtr d{nullptr, &mj_deleteData};
    int root_x_qpos = -1;
    int root_z_qpos = -1;
    int root_pitch_qpos = -1;
    int fork_qpos = -1;
    int front_spin_dof = -1;
    int rear_spin_dof = -1;
    int root_vx_dof = -1;
    int root_vz_dof = -1;
    int front_contact_geom = -1;
    int rear_contact_geom = -1;
};

WeldedFixture load_welded_fixture(const fs::path &xml_path, bool rough_uphill) {
    std::array<char, 1024> err{};
    mjModel *raw_m = mj_loadXML(xml_path.string().c_str(), nullptr, err.data(),
                                static_cast<int>(err.size()));
    require(raw_m != nullptr, std::string("mj_loadXML failed: ") + err.data());
    ModelPtr m(raw_m, &mj_deleteModel);
    require(m->nq == 27 && m->nv == 27, "welded.xml must have nq=nv=27");

    populate_welded_terrain(m.get(), rough_uphill);
    DataPtr d(mj_makeData(m.get()), &mj_deleteData);
    mj_resetData(m.get(), d.get());

    const std::span<const int> jnt_qposadr =
        model_access::readonly_buffer(m->jnt_qposadr, m->njnt, "jnt_qposadr");
    const std::span<const int> jnt_dofadr =
        model_access::readonly_buffer(m->jnt_dofadr, m->njnt, "jnt_dofadr");

    const auto find_qpos = [&](const char *jname) -> int {
        const int jid = mj_name2id(m.get(), mjOBJ_JOINT, jname);
        require(jid >= 0, std::string("missing joint: ") + jname);
        return jnt_qposadr[to_index(jid)];
    };
    const auto find_dof = [&](const char *jname) -> int {
        const int jid = mj_name2id(m.get(), mjOBJ_JOINT, jname);
        require(jid >= 0, std::string("missing joint: ") + jname);
        return jnt_dofadr[to_index(jid)];
    };

    const int root_x_qpos = find_qpos("root_x");
    const int root_z_qpos = find_qpos("root_z");
    const int root_pitch_qpos = find_qpos("root_pitch");
    const int fork_qpos = find_qpos("fork_travel");
    const int front_spin_dof = find_dof("front_wheel_spin");
    const int rear_spin_dof = find_dof("rear_wheel_spin");
    const int root_vx_dof = find_dof("root_x");
    const int root_vz_dof = find_dof("root_z");
    const int front_contact_geom = mj_name2id(raw_m, mjOBJ_GEOM, "geom_front_contact");
    const int rear_contact_geom = mj_name2id(raw_m, mjOBJ_GEOM, "geom_rear_contact");
    require(front_contact_geom >= 0 && rear_contact_geom >= 0,
            "welded.xml must contain geom_front_contact and geom_rear_contact");

    WeldedFixture fx{
        .m = std::move(m),
        .d = std::move(d),
        .root_x_qpos = root_x_qpos,
        .root_z_qpos = root_z_qpos,
        .root_pitch_qpos = root_pitch_qpos,
        .fork_qpos = fork_qpos,
        .front_spin_dof = front_spin_dof,
        .rear_spin_dof = rear_spin_dof,
        .root_vx_dof = root_vx_dof,
        .root_vz_dof = root_vz_dof,
        .front_contact_geom = front_contact_geom,
        .rear_contact_geom = rear_contact_geom,
    };
    mj_forward(fx.m.get(), fx.d.get());
    return fx;
}

struct TickPose {
    const char *label;
    double root_x_offset;
    double ground_drop_m;
    double root_vx;
    double root_vz;
    double front_spin_delta;
    double rear_spin_delta;
    bool expect_contact;
};

constexpr std::array<TickPose, 6> kParityTicks{{
    {.label = "loaded_rolling", .root_x_offset = 2.0, .ground_drop_m = -0.015, .root_vx = 5.0, .root_vz = -0.05, .front_spin_delta = 0.0, .rear_spin_delta = 0.0, .expect_contact = true},
    {.label = "loaded_slip", .root_x_offset = 2.008, .ground_drop_m = -0.018, .root_vx = 5.0, .root_vz = -0.10, .front_spin_delta = +4.0, .rear_spin_delta = -3.5, .expect_contact = true},
    {.label = "heavy_compression", .root_x_offset = 2.016, .ground_drop_m = -0.024, .root_vx = 6.2, .root_vz = -0.25, .front_spin_delta = +6.0, .rear_spin_delta = -4.5, .expect_contact = true},
    {.label = "branch_break_jump", .root_x_offset = 2.35, .ground_drop_m = -0.014, .root_vx = 6.5, .root_vz = -0.08, .front_spin_delta = +2.0, .rear_spin_delta = -2.0, .expect_contact = true},
    {.label = "airborne", .root_x_offset = 2.40, .ground_drop_m = +0.45, .root_vx = 6.5, .root_vz = +0.30, .front_spin_delta = 0.0, .rear_spin_delta = 0.0, .expect_contact = false},
    {.label = "re_touchdown", .root_x_offset = 2.45, .ground_drop_m = -0.016, .root_vx = 5.8, .root_vz = -0.20, .front_spin_delta = +3.0, .rear_spin_delta = -2.5, .expect_contact = true},
}};

void apply_tick_pose(WeldedFixture &fx, const TickPose &pose, bool rough_uphill) {
    const int nq = static_cast<int>(fx.m->nq);
    const int nv = static_cast<int>(fx.m->nv);
    const std::span<mjtNum> qpos = model_access::mutable_buffer(fx.d->qpos, nq, "qpos");
    const std::span<mjtNum> qvel = model_access::mutable_buffer(fx.d->qvel, nv, "qvel");

    std::ranges::fill(qpos, 0.0);
    std::ranges::fill(qvel, 0.0);

    const double gz = ground_profile_z(pose.root_x_offset, rough_uphill);
    qpos[to_index(fx.root_x_qpos)] = pose.root_x_offset;
    qpos[to_index(fx.root_z_qpos)] = gz + pose.ground_drop_m;
    qpos[to_index(fx.root_pitch_qpos)] = rough_uphill ? -0.10 : 0.0;
    qpos[to_index(fx.fork_qpos)] = 0.025;
    mj_forward(fx.m.get(), fx.d.get());

    // Shift root_z once so the lower wheel clears or penetrates the ground cleanly by ground_drop_m.
    const std::span<const int> geom_bodyid =
        model_access::readonly_buffer(fx.m->geom_bodyid, fx.m->ngeom, "geom_bodyid");
    const std::span<const mjtNum> geom_size =
        model_access::readonly_buffer(fx.m->geom_size, 3 * fx.m->ngeom, "geom_size");
    const std::span<const mjtNum> xpos =
        model_access::readonly_buffer(fx.d->xpos, 3 * fx.m->nbody, "xpos");
    const int f_body = geom_bodyid[to_index(fx.front_contact_geom)];
    const int r_body = geom_bodyid[to_index(fx.rear_contact_geom)];
    const double f_r = geom_size[3 * to_index(fx.front_contact_geom)];
    const double r_r = geom_size[3 * to_index(fx.rear_contact_geom)];
    const double f_gz = ground_profile_z(xpos[3 * to_index(f_body) + 0], rough_uphill);
    const double r_gz = ground_profile_z(xpos[3 * to_index(r_body) + 0], rough_uphill);
    const double f_clear = (xpos[3 * to_index(f_body) + 2] - f_r) - f_gz;
    const double r_clear = (xpos[3 * to_index(r_body) + 2] - r_r) - r_gz;
    const double min_clear = std::min(f_clear, r_clear);
    qpos[to_index(fx.root_z_qpos)] += (pose.ground_drop_m - min_clear);

    qvel[to_index(fx.root_vx_dof)] = pose.root_vx;
    qvel[to_index(fx.root_vz_dof)] = pose.root_vz;
    qvel[to_index(fx.front_spin_dof)] = pose.root_vx / f_r + pose.front_spin_delta;
    qvel[to_index(fx.rear_spin_dof)] = pose.root_vx / r_r + pose.rear_spin_delta;
    mj_forward(fx.m.get(), fx.d.get());
}

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
[[gnu::noinline]] std::unique_ptr<HostTyreUploadBundle> build_tyre_bundle_heap(
    const mjModel *m,
    const mjData *d,
    const nativecfg::TireConfig &cfg,
    int front_geom,
    int rear_geom
) {
    return std::make_unique<HostTyreUploadBundle>(
        build_gpu_tyre_model(m, d, cfg, front_geom, rear_geom));
}
NATIVE_DIAG_POP

void run_parity_suite(
    const fs::path &xml_path,
    bool rough_uphill,
    const nativecfg::TireConfig &cfg
) {
    WeldedFixture fx = load_welded_fixture(xml_path, rough_uphill);
    const int nv = static_cast<int>(fx.m->nv);
    const int nbody = static_cast<int>(fx.m->nbody);
    const int njnt = static_cast<int>(fx.m->njnt);

    const auto bundle_ptr = build_tyre_bundle_heap(
        fx.m.get(), fx.d.get(), cfg, fx.front_contact_geom, fx.rear_contact_geom);
    const HostTyreUploadBundle &bundle = *bundle_ptr;
    const bike::gpu::HostPackedTerrain terrain = bike::gpu::pack_upload_bundle_terrain(bundle);

    const bool is_dist = (cfg.backend == "distributed_2d_reference");
    std::unique_ptr<TireWriter> compliant_writer;
    std::unique_ptr<DistributedTireWriter> dist_writer;
    if (is_dist) {
        dist_writer = std::make_unique<DistributedTireWriter>(fx.m.get(), fx.d.get(), cfg);
    } else {
        compliant_writer = std::make_unique<TireWriter>(fx.m.get(), fx.d.get(), cfg);
    }

    std::vector<GpuCompliantWheelState> comp_state(2);
    std::vector<GpuStationState> dist_state(2ULL * to_index(kMaxGpuTyreStations));
    const float dt_f = 0.001f;
    const double dt_d = static_cast<double>(dt_f);

    for (std::size_t tick = 0; tick < kParityTicks.size(); ++tick) {
        const TickPose &pose = kParityTicks[tick];
        apply_tick_pose(fx, pose, rough_uphill);
        fx.d->time = static_cast<double>(tick + 1) * dt_d;

        std::vector<double> ref_qfrc(to_index(nv), 0.0);
        std::array<std::optional<TireDiagnostics>, 2> ref_diags{};
        std::array<std::optional<TireSnapshot>, 2> ref_snaps{};
        if (is_dist) {
            dist_writer->compute_into(fx.d.get(), dt_d, ref_qfrc);
            ref_diags = dist_writer->diagnostics();
            ref_snaps = dist_writer->snapshots();
        } else {
            compliant_writer->compute_into(fx.d.get(), dt_d, ref_qfrc);
            ref_diags = compliant_writer->diagnostics();
            ref_snaps = compliant_writer->snapshots();
        }

        std::vector<float> xpos_f(3U * to_index(nbody), 0.0f);
        std::vector<float> xmat_f(9U * to_index(nbody), 0.0f);
        std::vector<float> xanchor_f(3U * to_index(njnt), 0.0f);
        std::vector<float> xaxis_f(3U * to_index(njnt), 0.0f);
        std::vector<float> qvel_f(to_index(nv), 0.0f);
        std::vector<float> gpu_qfrc(to_index(nv), 0.0f);

        const std::span<const mjtNum> d_xpos =
            model_access::readonly_buffer(fx.d->xpos, 3LL * static_cast<mjtSize>(nbody), "xpos");
        const std::span<const mjtNum> d_xmat =
            model_access::readonly_buffer(fx.d->xmat, 9LL * static_cast<mjtSize>(nbody), "xmat");
        const std::span<const mjtNum> d_xanchor =
            model_access::readonly_buffer(fx.d->xanchor, 3LL * static_cast<mjtSize>(njnt), "xanchor");
        const std::span<const mjtNum> d_xaxis =
            model_access::readonly_buffer(fx.d->xaxis, 3LL * static_cast<mjtSize>(njnt), "xaxis");
        const std::span<const mjtNum> d_qvel =
            model_access::readonly_buffer(fx.d->qvel, nv, "qvel");
        for (std::size_t i = 0; i < xpos_f.size(); ++i) xpos_f[i] = static_cast<float>(d_xpos[i]);
        for (std::size_t i = 0; i < xmat_f.size(); ++i) xmat_f[i] = static_cast<float>(d_xmat[i]);
        for (std::size_t i = 0; i < xanchor_f.size(); ++i) {
            xanchor_f[i] = static_cast<float>(d_xanchor[i]);
            xaxis_f[i] = static_cast<float>(d_xaxis[i]);
        }
        for (std::size_t i = 0; i < qvel_f.size(); ++i) qvel_f[i] = static_cast<float>(d_qvel[i]);

        GpuTyreStepOutput out_sum{};
        ErrCode err = ErrCode::none;
        const Status st = gpu_step_both_wheels_tyre(
            &bundle.gpu_model,
            terrain.desc,
            terrain.f32.empty() ? nullptr : terrain.f32.data(),
            terrain.i32.empty() ? nullptr : terrain.i32.data(),
            1.0f,
            dt_f,
            xpos_f.data(),
            xmat_f.data(),
            xanchor_f.data(),
            xaxis_f.data(),
            qvel_f.data(),
            comp_state.data(),
            dist_state.data(),
            gpu_qfrc.data(),
            out_sum,
            err,
            true);
        require(st == Status::ok && err == ErrCode::none,
                std::format("gpu_step_both_wheels_tyre failed on tick '{}' ({})",
                            pose.label, rough_uphill ? "rough" : "flat"));

        double max_ref_qfrc = 0.0;
        for (const double v : ref_qfrc) {
            max_ref_qfrc = std::max(max_ref_qfrc, std::abs(v));
        }
        if (pose.expect_contact) {
            require(max_ref_qfrc > 20.0,
                    std::format("expected non-trivial contact qfrc on tick '{}'", pose.label));
        } else {
            require(max_ref_qfrc == 0.0, "airborne tick must yield zero qfrc");
        }

        const double qfrc_tol = 5e-5 * std::max(1.0, max_ref_qfrc);
        for (int i = 0; i < nv; ++i) {
            const double diff = std::abs(static_cast<double>(gpu_qfrc[to_index(i)]) - ref_qfrc[to_index(i)]);
            require(diff <= qfrc_tol,
                    std::format("qfrc parity mismatch tick='{}' dof={} diff={:.6g} tol={:.6g}",
                                pose.label, i, diff, qfrc_tol));
        }

        const std::span sum_wheels(out_sum.wheels);
        const std::span model_wheels(bundle.gpu_model.wheels);
        const std::span active_cnts(out_sum.active_station_counts);
        for (std::size_t side = 0; side < 2; ++side) {
            const std::optional<TireDiagnostics> &diag_opt = ref_diags[side];
            const std::optional<TireSnapshot> &snap_opt = ref_snaps[side];
            if (!diag_opt.has_value() || !snap_opt.has_value()) {
                throw std::runtime_error("reference diagnostics/snapshot present");
            }
            const double ref_n = diag_opt->normal_load_n;
            const double gpu_n = static_cast<double>(sum_wheels[side].normal_load_n);
            const double n_tol = 5e-5 * std::max(1.0, std::abs(ref_n));
            require(std::abs(gpu_n - ref_n) <= n_tol,
                    std::format("normal load mismatch tick='{}' side={} gpu={:.6g} ref={:.6g}",
                                pose.label, side, gpu_n, ref_n));

            const float nom_r = model_wheels[side].radius_m;
            const float eff_r = sum_wheels[side].effective_radius_m;
            const double ref_eff_r = snap_opt->effective_radius_m;
            const bool wheel_airborne = snap_opt->patches.empty();
            require(std::isfinite(eff_r), "effective_radius_m must be finite");
            if (!pose.expect_contact) {
                require(wheel_airborne, "airborne tick must have no contact patches");
            }
            if (wheel_airborne) {
                require(ref_eff_r == 0.0 && eff_r == 0.0f,
                        std::format("airborne effective_radius_m must be exact 0.0f tick='{}' side={} gpu={} ref={}",
                                    pose.label, side, eff_r, ref_eff_r));
            } else {
                require(eff_r > 0.05f && eff_r <= nom_r + 1e-4f,
                        "effective_radius_m lies inside physical wheel bounds");
                const double r_tol = 5e-5 * std::max(1.0, std::abs(ref_eff_r));
                require(std::abs(static_cast<double>(eff_r) - ref_eff_r) <= r_tol,
                        std::format("effective_radius_m parity mismatch tick='{}' side={} gpu={:.6g} ref={:.6g}",
                                    pose.label, side, static_cast<double>(eff_r), ref_eff_r));
            }

            if (is_dist) {
                require(active_cnts[side] == diag_opt->active_station_count,
                        std::format("active_station_count mismatch tick='{}' side={} gpu={} ref={}",
                                    pose.label, side, active_cnts[side],
                                    diag_opt->active_station_count));
            }
        }
    }
}

void test_compliant_flat_parity(const fs::path &xml) {
    run_parity_suite(xml, false, bike::gpu::make_default_compliant_tire_config());
}

void test_compliant_rough_uphill_parity(const fs::path &xml) {
    run_parity_suite(xml, true, bike::gpu::make_default_compliant_tire_config());
}

void test_distributed_flat_and_rough_parity(const fs::path &xml) {
    const nativecfg::TireConfig dcfg = bike::gpu::make_default_distributed_tire_config(64);
    run_parity_suite(xml, false, dcfg);
    run_parity_suite(xml, true, dcfg);
}

void test_airborne_effective_radius_zero(const fs::path &xml) {
    WeldedFixture fx = load_welded_fixture(xml, false);
    const TickPose airborne_pose{
        .label = "airborne_explicit",
        .root_x_offset = 2.2,
        .ground_drop_m = +0.50,
        .root_vx = 5.0,
        .root_vz = +0.20,
        .front_spin_delta = 1.0,
        .rear_spin_delta = -1.0,
        .expect_contact = false,
    };
    apply_tick_pose(fx, airborne_pose, false);
    const int nv = static_cast<int>(fx.m->nv);
    const int nbody = static_cast<int>(fx.m->nbody);
    const int njnt = static_cast<int>(fx.m->njnt);

    std::vector<float> xpos_f(3U * to_index(nbody), 0.0f);
    std::vector<float> xmat_f(9U * to_index(nbody), 0.0f);
    std::vector<float> xanchor_f(3U * to_index(njnt), 0.0f);
    std::vector<float> xaxis_f(3U * to_index(njnt), 0.0f);
    std::vector<float> qvel_f(to_index(nv), 0.0f);
    const std::span<const mjtNum> d_xpos = model_access::readonly_buffer(fx.d->xpos, 3LL * static_cast<mjtSize>(nbody));
    const std::span<const mjtNum> d_xmat = model_access::readonly_buffer(fx.d->xmat, 9LL * static_cast<mjtSize>(nbody));
    const std::span<const mjtNum> d_xanchor = model_access::readonly_buffer(fx.d->xanchor, 3LL * static_cast<mjtSize>(njnt));
    const std::span<const mjtNum> d_xaxis = model_access::readonly_buffer(fx.d->xaxis, 3LL * static_cast<mjtSize>(njnt));
    const std::span<const mjtNum> d_qvel = model_access::readonly_buffer(fx.d->qvel, nv);
    for (std::size_t i = 0; i < xpos_f.size(); ++i) xpos_f[i] = static_cast<float>(d_xpos[i]);
    for (std::size_t i = 0; i < xmat_f.size(); ++i) xmat_f[i] = static_cast<float>(d_xmat[i]);
    for (std::size_t i = 0; i < xanchor_f.size(); ++i) {
        xanchor_f[i] = static_cast<float>(d_xanchor[i]);
        xaxis_f[i] = static_cast<float>(d_xaxis[i]);
    }
    for (std::size_t i = 0; i < qvel_f.size(); ++i) qvel_f[i] = static_cast<float>(d_qvel[i]);

    std::vector<nativecfg::TireConfig> configs;
    configs.push_back(bike::gpu::make_default_compliant_tire_config());
    configs.push_back(bike::gpu::make_default_distributed_tire_config(64));
    for (const nativecfg::TireConfig &cfg : configs) {
        const auto bundle_ptr = build_tyre_bundle_heap(
            fx.m.get(), fx.d.get(), cfg, fx.front_contact_geom, fx.rear_contact_geom);
        const HostTyreUploadBundle &bundle = *bundle_ptr;
        const bike::gpu::HostPackedTerrain terrain = bike::gpu::pack_upload_bundle_terrain(bundle);

        std::vector<GpuCompliantWheelState> cs(2);
        std::vector<GpuStationState> ds(2ULL * to_index(kMaxGpuTyreStations));
        std::vector<float> qfrc(to_index(nv), 0.0f);
        GpuTyreStepOutput sum{};
        ErrCode err = ErrCode::none;
        require(gpu_step_both_wheels_tyre(
                    &bundle.gpu_model, terrain.desc, terrain.f32.data(), terrain.i32.data(),
                    1.0f, 0.001f, xpos_f.data(), xmat_f.data(), xanchor_f.data(), xaxis_f.data(),
                    qvel_f.data(), cs.data(), ds.data(), qfrc.data(), sum, err, true) == Status::ok,
                "airborne gpu_step_both_wheels_tyre succeeds");
        const std::span sum_wheels(sum.wheels);
        const std::span active_cnts(sum.active_station_counts);
        for (std::size_t side = 0; side < 2; ++side) {
            require(sum_wheels[side].normal_load_n == 0.0f, "airborne normal_load_n == 0");
            require(sum_wheels[side].effective_radius_m == 0.0f,
                    "airborne effective_radius_m == 0.0f");
            require(active_cnts[side] == 0, "airborne active_station_counts == 0");
        }
    }
}

void test_stiffness_scale_and_validation(const fs::path &xml) {
    WeldedFixture fx = load_welded_fixture(xml, false);
    const TickPose deep_pose{
        .label = "stiffness_test",
        .root_x_offset = 2.0,
        .ground_drop_m = -0.035,
        .root_vx = 5.0,
        .root_vz = -0.05,
        .front_spin_delta = 0.25,
        .rear_spin_delta = -0.25,
        .expect_contact = true,
    };
    apply_tick_pose(fx, deep_pose, false);
    const int nv = static_cast<int>(fx.m->nv);
    const int nbody = static_cast<int>(fx.m->nbody);
    const int njnt = static_cast<int>(fx.m->njnt);

    std::vector<float> xpos_f(3U * to_index(nbody), 0.0f);
    std::vector<float> xmat_f(9U * to_index(nbody), 0.0f);
    std::vector<float> xanchor_f(3U * to_index(njnt), 0.0f);
    std::vector<float> xaxis_f(3U * to_index(njnt), 0.0f);
    std::vector<float> qvel_f(to_index(nv), 0.0f);
    const std::span<const mjtNum> d_xpos = model_access::readonly_buffer(fx.d->xpos, 3LL * static_cast<mjtSize>(nbody));
    const std::span<const mjtNum> d_xmat = model_access::readonly_buffer(fx.d->xmat, 9LL * static_cast<mjtSize>(nbody));
    const std::span<const mjtNum> d_xanchor = model_access::readonly_buffer(fx.d->xanchor, 3LL * static_cast<mjtSize>(njnt));
    const std::span<const mjtNum> d_xaxis = model_access::readonly_buffer(fx.d->xaxis, 3LL * static_cast<mjtSize>(njnt));
    const std::span<const mjtNum> d_qvel = model_access::readonly_buffer(fx.d->qvel, nv);
    for (std::size_t i = 0; i < xpos_f.size(); ++i) xpos_f[i] = static_cast<float>(d_xpos[i]);
    for (std::size_t i = 0; i < xmat_f.size(); ++i) xmat_f[i] = static_cast<float>(d_xmat[i]);
    for (std::size_t i = 0; i < xanchor_f.size(); ++i) {
        xanchor_f[i] = static_cast<float>(d_xanchor[i]);
        xaxis_f[i] = static_cast<float>(d_xaxis[i]);
    }
    for (std::size_t i = 0; i < qvel_f.size(); ++i) qvel_f[i] = static_cast<float>(d_qvel[i]);

    std::vector<nativecfg::TireConfig> configs;
    configs.push_back(bike::gpu::make_default_compliant_tire_config());
    configs.push_back(bike::gpu::make_default_distributed_tire_config(64));
    for (const nativecfg::TireConfig &cfg : configs) {
        const auto bundle_ptr = build_tyre_bundle_heap(
            fx.m.get(), fx.d.get(), cfg, fx.front_contact_geom, fx.rear_contact_geom);
        const HostTyreUploadBundle &bundle = *bundle_ptr;
        const bike::gpu::HostPackedTerrain terrain = bike::gpu::pack_upload_bundle_terrain(bundle);

        std::vector<GpuCompliantWheelState> c1(2);
        std::vector<GpuCompliantWheelState> c2(2);
        std::vector<GpuStationState> d1(2ULL * to_index(kMaxGpuTyreStations));
        std::vector<GpuStationState> d2(2ULL * to_index(kMaxGpuTyreStations));
        std::vector<float> q1(to_index(nv), 0.0f);
        std::vector<float> q2(to_index(nv), 0.0f);
        GpuTyreStepOutput s1{};
        GpuTyreStepOutput s2{};
        ErrCode err = ErrCode::none;

        require(gpu_step_both_wheels_tyre(
                    &bundle.gpu_model, terrain.desc, terrain.f32.data(), terrain.i32.data(),
                    1.0f, 0.001f, xpos_f.data(), xmat_f.data(), xanchor_f.data(), xaxis_f.data(),
                    qvel_f.data(), c1.data(), d1.data(), q1.data(), s1, err) == Status::ok,
                "baseline scale=1.0 succeeds");
        require(gpu_step_both_wheels_tyre(
                    &bundle.gpu_model, terrain.desc, terrain.f32.data(), terrain.i32.data(),
                    1.20f, 0.001f, xpos_f.data(), xmat_f.data(), xanchor_f.data(), xaxis_f.data(),
                    qvel_f.data(), c2.data(), d2.data(), q2.data(), s2, err) == Status::ok,
                "scaled scale=1.20 succeeds");
        const std::span s1_wheels(s1.wheels);
        const std::span s2_wheels(s2.wheels);
        for (std::size_t side = 0; side < 2; ++side) {
            const float expected_load = 1.20f * s1_wheels[side].normal_load_n;
            require(s1_wheels[side].normal_load_n > 50.0f &&
                        std::abs(s2_wheels[side].normal_load_n - expected_load) <=
                            1e-4f * expected_load,
                    "tire_stiff_scale=1.20 scales normal_load_n proportionally");
        }
        for (const int spin_dof : {fx.front_spin_dof, fx.rear_spin_dof}) {
            const float tau1 = q1[to_index(spin_dof)];
            const float tau2 = q2[to_index(spin_dof)];
            const float expected_tau = 1.20f * tau1;
            require(std::abs(tau1) > 0.5f &&
                        std::abs(tau2 - expected_tau) <= 1e-4f * std::abs(expected_tau),
                    "tire_stiff_scale=1.20 scales tangential wheel spin qfrc proportionally");
        }
    }

    // Bounds rejection checks
    const auto bundle_ptr = build_tyre_bundle_heap(
        fx.m.get(), fx.d.get(), bike::gpu::make_default_compliant_tire_config(),
        fx.front_contact_geom, fx.rear_contact_geom);
    const HostTyreUploadBundle &bundle = *bundle_ptr;
    const bike::gpu::HostPackedTerrain terrain = bike::gpu::pack_upload_bundle_terrain(bundle);
    ErrCode err = ErrCode::none;

    {
        const auto bad_model = std::make_unique<GpuTyreModel>(bundle.gpu_model);
        const std::span bad_wheels(bad_model->wheels);
        const std::span bad_joints(bad_wheels[0].chain.joints);
        bad_joints[0].joint_id = bad_model->njnt;
        require(validate_gpu_tyre_model_device(bad_model.get(), 0, 0, 0, err) != Status::ok,
                "reject joint_id >= njnt");
    }
    {
        const auto bad_model = std::make_unique<GpuTyreModel>(bundle.gpu_model);
        bad_model->backend = TyreBackendKind::distributed_2d_reference;
        bad_model->station_count = static_cast<i32>(kMaxGpuTyreStations) + 1;
        require(validate_gpu_tyre_model_device(bad_model.get(), 0, 0, 0, err) != Status::ok,
                "reject station_count > 128");
    }
    {
        auto bad_f32 = terrain.f32;
        bad_f32[to_index(terrain.desc.px_offset + 5)] =
            bad_f32[to_index(terrain.desc.px_offset + 4)];
        require(validate_terrain_table_desc(
                    terrain.desc, bad_f32.data(), terrain.i32.data(), true, false, true, err) !=
                    Status::ok,
                "reject non-monotonic px");
    }
}

[[gnu::noinline]] void verify_metal_writers_parity(
    WeldedFixture &fx,
    const HostTyreUploadBundle &bundle,
    const bike::gpu::HostPackedTerrain &terrain,
    float dt_f,
    std::string_view label
) {
#if defined(__APPLE__)
    const auto idx = bike::gpu::assets::resolve_gpu_model_indices(fx.m.get());
    const int nq = idx.nq;
    const int nv = idx.nv;
    const int nu = idx.nu;
    const int nbody = idx.nbody;
    const int njnt = static_cast<int>(fx.m->njnt);

    fs::path gpu_dir;
#ifdef BIKE_REPO_ROOT
    gpu_dir = fs::path(BIKE_REPO_ROOT) / "native" / "src" / "gpu";
#else
    gpu_dir = fs::path("native") / "src" / "gpu";
#endif
    require(bike_metal_init(gpu_dir.string().c_str()) == 0, "bike_metal_init failed");

    auto cfg = std::make_unique<bike::gpu::GpuWorldConfig>();
    cfg->suspension = bike::gpu::gpu_make_backend_default_suspension_params(bundle.gpu_model.backend);
    cfg->resistance = bike::gpu::gpu_make_default_resistance_params();
    cfg->resistance.crr = 0.0f;
    cfg->resistance.cda_m2 = 0.0f;
    cfg->rider = bike::gpu::gpu_make_default_rider_params();
    cfg->rider.rider_enabled = 0;
    cfg->pedal_demand = 0.0f;
    cfg->front_brake_demand = 0.0f;
    cfg->rear_brake_demand = 0.0f;

    auto ws_host = std::make_unique<bike::gpu::GpuWorldState>();
    auto ws_mtl = std::make_unique<bike::gpu::GpuWorldState>();
    const auto pneum_scratch = std::make_unique<GpuPneumaticStepScratch<float>>();

    std::vector<float> qpos_f(to_index(nq), 0.0f);
    std::vector<float> qvel_f(to_index(nv), 0.0f);
    std::vector<float> xpos_f(3U * to_index(nbody), 0.0f);
    std::vector<float> xmat_f(9U * to_index(nbody), 0.0f);
    std::vector<float> xanchor_f(3U * to_index(njnt), 0.0f);
    std::vector<float> xaxis_f(3U * to_index(njnt), 0.0f);
    std::vector<float> qfrc_host(to_index(nv), 0.0f);
    std::vector<float> qfrc_mtl(to_index(nv), 0.0f);
    std::vector<float> ctrl_host(to_index(nu), 0.0f);
    std::vector<float> ctrl_mtl(to_index(nu), 0.0f);
    std::vector<float> tr_host(static_cast<std::size_t>(2 * fx.m->ntendon), 0.0f);
    std::vector<float> tr_mtl(static_cast<std::size_t>(2 * fx.m->ntendon), 0.0f);
    const int tr_stride = 2 * static_cast<int>(fx.m->ntendon);

    apply_tick_pose(fx, kParityTicks[0], false);
    const auto copy_state = [&]() {
        const auto dqpos = model_access::readonly_buffer(fx.d->qpos, nq, "qpos");
        const auto dqvel = model_access::readonly_buffer(fx.d->qvel, nv, "qvel");
        const auto dxpos = model_access::readonly_buffer(fx.d->xpos, 3LL * static_cast<mjtSize>(nbody));
        const auto dxmat = model_access::readonly_buffer(fx.d->xmat, 9LL * static_cast<mjtSize>(nbody));
        const auto dxanc = model_access::readonly_buffer(fx.d->xanchor, 3LL * static_cast<mjtSize>(njnt));
        const auto dxax = model_access::readonly_buffer(fx.d->xaxis, 3LL * static_cast<mjtSize>(njnt));
        for (int i = 0; i < nq; ++i) qpos_f[to_index(i)] = static_cast<float>(dqpos[to_index(i)]);
        for (int i = 0; i < nv; ++i) qvel_f[to_index(i)] = static_cast<float>(dqvel[to_index(i)]);
        for (std::size_t i = 0; i < xpos_f.size(); ++i) xpos_f[i] = static_cast<float>(dxpos[i]);
        for (std::size_t i = 0; i < xmat_f.size(); ++i) xmat_f[i] = static_cast<float>(dxmat[i]);
        for (std::size_t i = 0; i < xanchor_f.size(); ++i) {
            xanchor_f[i] = static_cast<float>(dxanc[i]);
            xaxis_f[i] = static_cast<float>(dxax[i]);
        }
    };
    copy_state();

    const float crank_q0 = (idx.crank_spin_qpos >= 0) ? qpos_f[to_index(idx.crank_spin_qpos)] : 0.0f;
    const float rear_q0 = (idx.rear_wheel_spin_qpos >= 0) ? qpos_f[to_index(idx.rear_wheel_spin_qpos)] : 0.0f;
    const bool has_ds = (idx.drive_shaft_spin_qpos >= 0 && idx.drive_shaft_spin_qpos < idx.nq);
    const float ds_q0 = has_ds ? qpos_f[to_index(idx.drive_shaft_spin_qpos)] : 0.0f;
    require(bike::gpu::gpu_init_world_state(
                *ws_host, cfg->drivetrain, crank_q0, rear_q0, ds_q0, has_ds) == Status::ok,
            "host init world state failed");
    require(bike_metal_launch_init_states(1, &idx, cfg.get(), 1, ws_mtl.get(), qpos_f.data()) == 0,
            "metal init states failed");

    const float *tf32 = terrain.f32.empty() ? nullptr : terrain.f32.data();
    const i32 *ti32 = terrain.i32.empty() ? nullptr : terrain.i32.data();
    for (int step = 0; step < 3; ++step) {
        const auto &pose = kParityTicks[to_index(step)];
        apply_tick_pose(fx, pose, false);
        copy_state();

        require(bike::gpu::gpu_step_world_writers(
                    0, dt_f, idx, &bundle.gpu_model, terrain.desc, tf32, ti32,
                    cfg.get(), 1, ws_host.get(),
                    qpos_f.data(), qvel_f.data(), xpos_f.data(), xmat_f.data(),
                    xanchor_f.data(), xaxis_f.data(),
                    qfrc_host.data(), ctrl_host.data(), tr_host.data(), tr_stride,
                    false, pneum_scratch.get()) == Status::ok,
                "host gpu_step_world_writers failed");
        require(bike_metal_launch_writers(
                    1, dt_f, &idx, &bundle.gpu_model, &terrain.desc, tf32, ti32,
                    cfg.get(), 1, ws_mtl.get(),
                    qpos_f.data(), qvel_f.data(), xpos_f.data(), xmat_f.data(),
                    xanchor_f.data(), xaxis_f.data(),
                    qfrc_mtl.data(), ctrl_mtl.data(), tr_mtl.data(), tr_stride) == 0,
                "bike_metal_launch_writers failed");

        for (int i = 0; i < nv; ++i) {
            const float h = qfrc_host[to_index(i)];
            const float m = qfrc_mtl[to_index(i)];
            const float tol = 2e-5f * std::max(1.0f, std::abs(h));
            const auto h_bits = std::bit_cast<std::int32_t>(h);
            const auto m_bits = std::bit_cast<std::int32_t>(m);
            const bool same_sign = ((h_bits < 0) == (m_bits < 0));
            const std::uint32_t ulp = same_sign
                ? static_cast<std::uint32_t>(std::abs(h_bits - m_bits))
                : std::numeric_limits<std::uint32_t>::max();
            require(ulp <= 4U || std::abs(h - m) <= tol,
                    std::format("Host vs Metal qfrc mismatch ({} step={} dof={}): host={} mtl={}",
                                label, step, i, h, m));
        }
    }
#else
    (void)fx;
    (void)bundle;
    (void)terrain;
    (void)dt_f;
    (void)label;
#endif
}

[[gnu::noinline]] void test_tabulated_compliant_parity(const fs::path &xml) {
    nativecfg::TireConfig tab_cfg = bike::gpu::make_default_compliant_tire_config();
    require(tab_cfg.front.material.has_value() && tab_cfg.rear.material.has_value(),
            "compliant cfg must have material");
    const nativecfg::TireTable tbl{
        .deflection_m = {0.0, 0.015, 0.030, 0.060},
        .force_n = {0.0, 1200.0, 3000.0, 7200.0},
    };
    tab_cfg.front.material->table = tbl;
    tab_cfg.rear.material->table = tbl;
    tab_cfg.front.material->valid_load_range_n = {0.0, 6000.0};
    tab_cfg.rear.material->valid_load_range_n = {0.0, 6000.0};

    run_parity_suite(xml, false, tab_cfg);

    WeldedFixture fx = load_welded_fixture(xml, false);
    apply_tick_pose(fx, kParityTicks[2], false);
    const auto bundle_ptr = build_tyre_bundle_heap(
        fx.m.get(), fx.d.get(), tab_cfg, fx.front_contact_geom, fx.rear_contact_geom);
    const bike::gpu::HostPackedTerrain terrain = bike::gpu::pack_upload_bundle_terrain(*bundle_ptr);
    const std::span<const TyreWheelSpec<float>, 2> wheels(bundle_ptr->gpu_model.wheels);
    require(wheels[0].compliant.use_tabulated == 1 && wheels[1].compliant.use_tabulated == 1,
            "TabulatedRadial use_tabulated == 1");
    verify_metal_writers_parity(fx, *bundle_ptr, terrain, 0.001f, "tabulated_compliant");
}

[[gnu::noinline]] void test_distributed_stage_a_transitions_parity(const fs::path &xml) {
    nativecfg::TireConfig dcfg = bike::gpu::make_default_distributed_tire_config(64);
    dcfg.surface_mode = "track";
    dcfg.surface_map = nativecfg::SurfaceMap{
        .surface = bike::gpu::make_surface_spec_from_label(0), // asphalt
        .sections = {
            nativecfg::SurfaceSection{
                .start_m = 2.65,
                .end_m = 3.40,
                .surface = bike::gpu::make_surface_spec_from_label(4), // scree: crr_delta=0.008, load_sens=0.15
                .blend_m = 0.30,
            },
        },
    };
    run_parity_suite(xml, false, dcfg);

    WeldedFixture fx = load_welded_fixture(xml, false);
    apply_tick_pose(fx, kParityTicks[2], false); // root_x = 2.016 inside transition blend
    const auto bundle_ptr = build_tyre_bundle_heap(
        fx.m.get(), fx.d.get(), dcfg, fx.front_contact_geom, fx.rear_contact_geom);
    const bike::gpu::HostPackedTerrain terrain = bike::gpu::pack_upload_bundle_terrain(*bundle_ptr);
    const auto nv = static_cast<std::size_t>(fx.m->nv);
    const auto nbody = static_cast<std::size_t>(fx.m->nbody);
    const auto njnt = static_cast<std::size_t>(fx.m->njnt);
    std::vector<float> xpos_f(3U * nbody), xmat_f(9U * nbody), xanchor_f(3U * njnt), xaxis_f(3U * njnt), qvel_f(nv), qfrc_f(nv);
    const auto dxpos = model_access::readonly_buffer(fx.d->xpos, 3LL * static_cast<mjtSize>(nbody));
    const auto dxmat = model_access::readonly_buffer(fx.d->xmat, 9LL * static_cast<mjtSize>(nbody));
    const auto dxanc = model_access::readonly_buffer(fx.d->xanchor, 3LL * static_cast<mjtSize>(njnt));
    const auto dxax = model_access::readonly_buffer(fx.d->xaxis, 3LL * static_cast<mjtSize>(njnt));
    const auto dqvel = model_access::readonly_buffer(fx.d->qvel, static_cast<int>(nv));
    for (std::size_t i = 0; i < xpos_f.size(); ++i) xpos_f[i] = static_cast<float>(dxpos[i]);
    for (std::size_t i = 0; i < xmat_f.size(); ++i) xmat_f[i] = static_cast<float>(dxmat[i]);
    for (std::size_t i = 0; i < xanchor_f.size(); ++i) {
        xanchor_f[i] = static_cast<float>(dxanc[i]);
        xaxis_f[i] = static_cast<float>(dxax[i]);
    }
    for (std::size_t i = 0; i < qvel_f.size(); ++i) qvel_f[i] = static_cast<float>(dqvel[i]);

    std::vector<GpuCompliantWheelState> cs(2);
    std::vector<GpuStationState> ds(2ULL * to_index(kMaxGpuTyreStations));
    GpuTyreStepOutput sum{};
    ErrCode err = ErrCode::none;
    require(gpu_step_both_wheels_tyre(
                &bundle_ptr->gpu_model, terrain.desc, terrain.f32.data(), terrain.i32.data(),
                1.0f, 0.001f, xpos_f.data(), xmat_f.data(), xanchor_f.data(), xaxis_f.data(),
                qvel_f.data(), cs.data(), ds.data(), qfrc_f.data(), sum, err, true) == Status::ok,
            "distributed Stage A step succeeds");
    require(std::abs(sum.rolling_moment_nm) > 1e-4f,
            "distributed Stage A rolling_moment_nm must be non-zero");
    require(sum.brush_and_rolling_loss_j > 0.0f,
            "distributed Stage A brush_and_rolling_loss_j must be > 0");
    verify_metal_writers_parity(fx, *bundle_ptr, terrain, 0.001f, "distributed_stage_a");
}

[[gnu::noinline]] void test_pneumatic_fast_and_detailed_parity(const fs::path &xml) {
    for (const bool detailed : {false, true}) {
        WeldedFixture fx = load_welded_fixture(xml, false);
        const std::span<mjtNum> gsize =
            model_access::mutable_buffer(fx.m->geom_size, 3LL * static_cast<mjtSize>(fx.m->ngeom), "geom_size");
        gsize[3U * to_index(fx.front_contact_geom) + 0U] = 0.379;
        gsize[3U * to_index(fx.rear_contact_geom) + 0U] = 0.360;
        const std::span<int> gct =
            model_access::mutable_buffer(fx.m->geom_contype, fx.m->ngeom, "geom_contype");
        const std::span<int> gca =
            model_access::mutable_buffer(fx.m->geom_conaffinity, fx.m->ngeom, "geom_conaffinity");
        const std::span<const int> gbody =
            model_access::readonly_buffer(fx.m->geom_bodyid, fx.m->ngeom, "geom_bodyid");
        const int f_body = gbody[to_index(fx.front_contact_geom)];
        const int r_body = gbody[to_index(fx.rear_contact_geom)];
        for (int g = 0; g < fx.m->ngeom; ++g) {
            if (gbody[to_index(g)] == f_body || gbody[to_index(g)] == r_body) {
                gct[to_index(g)] = 0;
                gca[to_index(g)] = 0;
            }
        }
        mj_forward(fx.m.get(), fx.d.get());

        auto pcfg_ptr = std::make_unique<nativecfg::TireConfig>(
            bike::gpu::make_default_pneumatic_tire_config(detailed));
        nativecfg::TireConfig &pcfg = *pcfg_ptr;
        pcfg.surface_mode = "track";
        pcfg.pneumatic->surface.reset();
        pcfg.surface_map = nativecfg::SurfaceMap{
            .surface = bike::gpu::make_surface_spec_from_label(1), // hardpack
            .sections = {
                nativecfg::SurfaceSection{
                    .start_m = 2.02,
                    .end_m = 2.60,
                    .surface = bike::gpu::make_surface_spec_from_label(4), // scree
                    .blend_m = 0.25,
                },
            },
        };

        apply_tick_pose(fx, kParityTicks[0], false);
        const auto bundle_ptr = build_tyre_bundle_heap(
            fx.m.get(), fx.d.get(), pcfg, fx.front_contact_geom, fx.rear_contact_geom);
        const HostTyreUploadBundle &bundle = *bundle_ptr;
        const bike::gpu::HostPackedTerrain terrain = bike::gpu::pack_upload_bundle_terrain(bundle);

        pcfg.pneumatic->profile.x0_m = static_cast<double>(bundle.pl_px.front());
        pcfg.pneumatic->profile.dx_m = static_cast<double>(bundle.gpu_model.road_dx_m);
        pcfg.pneumatic->profile.z_m.resize(bundle.pl_pz.size());
        for (std::size_t i = 0; i < bundle.pl_pz.size(); ++i) {
            pcfg.pneumatic->profile.z_m[i] = static_cast<double>(bundle.pl_pz[i]);
        }

        const auto pneu_writer = std::make_unique<PneumaticTireWriter>(
            fx.m.get(), fx.d.get(), std::move(pcfg));

        const int nv = static_cast<int>(fx.m->nv);
        const int nbody = static_cast<int>(fx.m->nbody);
        const int njnt = static_cast<int>(fx.m->njnt);
        std::vector<GpuCompliantWheelState> comp_state(2);
        std::vector<GpuStationState> dist_state(2ULL * to_index(kMaxGpuTyreStations));
        auto pneu_states = std::make_unique<std::array<GpuPneumaticWheelState<float>, 2>>();
        const auto pneu_scratch = std::make_unique<GpuPneumaticStepScratch<float>>();

        const float dt_f = detailed ? 0.00025f : 0.0005f;
        const double dt_d = static_cast<double>(dt_f);
        float max_radial_e = 0.0f;
        float max_maxwell_e = 0.0f;
        float max_shear_e = 0.0f;
        float sum_radial_loss = 0.0f;
        float sum_brush_loss = 0.0f;

        for (int step = 0; step < 6; ++step) {
            TickPose pose = kParityTicks[to_index(step % 3)];
            pose.root_x_offset = 2.0 + 0.015 * static_cast<double>(step);
            pose.front_spin_delta = 3.0;
            pose.rear_spin_delta = -3.0;
            apply_tick_pose(fx, pose, false);
            fx.d->time = static_cast<double>(step + 1) * dt_d;

            std::vector<double> ref_qfrc(to_index(nv), 0.0);
            pneu_writer->compute_into(fx.d.get(), dt_d, ref_qfrc);

            std::vector<float> xpos_f(3U * to_index(nbody));
            std::vector<float> xmat_f(9U * to_index(nbody));
            std::vector<float> xanchor_f(3U * to_index(njnt));
            std::vector<float> xaxis_f(3U * to_index(njnt));
            std::vector<float> qvel_f(to_index(nv));
            std::vector<float> gpu_qfrc(to_index(nv), 0.0f);
            const auto dxpos = model_access::readonly_buffer(fx.d->xpos, 3LL * static_cast<mjtSize>(nbody));
            const auto dxmat = model_access::readonly_buffer(fx.d->xmat, 9LL * static_cast<mjtSize>(nbody));
            const auto dxanc = model_access::readonly_buffer(fx.d->xanchor, 3LL * static_cast<mjtSize>(njnt));
            const auto dxax = model_access::readonly_buffer(fx.d->xaxis, 3LL * static_cast<mjtSize>(njnt));
            const auto dqvel = model_access::readonly_buffer(fx.d->qvel, nv);
            for (std::size_t i = 0; i < xpos_f.size(); ++i) xpos_f[i] = static_cast<float>(dxpos[i]);
            for (std::size_t i = 0; i < xmat_f.size(); ++i) xmat_f[i] = static_cast<float>(dxmat[i]);
            for (std::size_t i = 0; i < xanchor_f.size(); ++i) {
                xanchor_f[i] = static_cast<float>(dxanc[i]);
                xaxis_f[i] = static_cast<float>(dxax[i]);
            }
            for (std::size_t i = 0; i < qvel_f.size(); ++i) qvel_f[i] = static_cast<float>(dqvel[i]);

            GpuTyreStepOutput sum{};
            ErrCode err = ErrCode::none;
            const Status st = gpu_step_both_wheels_tyre(
                &bundle.gpu_model, terrain.desc, terrain.f32.data(), terrain.i32.data(),
                1.0f, dt_f, xpos_f.data(), xmat_f.data(), xanchor_f.data(), xaxis_f.data(),
                qvel_f.data(), comp_state.data(), dist_state.data(), gpu_qfrc.data(),
                sum, err, true, pneu_states->data(), pneu_scratch.get());
            require(st == Status::ok && err == ErrCode::none,
                    std::format("pneumatic gpu_step_both_wheels_tyre failed detailed={} step={}",
                                detailed, step));

            double max_ref = 0.0;
            for (const double v : ref_qfrc) {
                max_ref = std::max(max_ref, std::abs(v));
            }
            require(max_ref > 20.0, "pneumatic contact tick must yield non-trivial qfrc");
            const double qfrc_tol = 2e-4 * std::max(1.0, max_ref);
            for (int i = 0; i < nv; ++i) {
                const double diff = std::abs(static_cast<double>(gpu_qfrc[to_index(i)]) - ref_qfrc[to_index(i)]);
                require(diff <= qfrc_tol,
                        std::format("pneumatic qfrc parity mismatch detailed={} step={} dof={} diff={:.6g} tol={:.6g}",
                                    detailed, step, i, diff, qfrc_tol));
            }

            max_radial_e = std::max(max_radial_e, sum.radial_energy_j);
            max_maxwell_e = std::max(max_maxwell_e, sum.maxwell_energy_j);
            max_shear_e = std::max(max_shear_e, sum.shear_energy_j);
            sum_radial_loss += sum.radial_loss_j;
            sum_brush_loss += sum.brush_and_rolling_loss_j;

            const std::span<const GpuPneumaticWheelState<float>, 2> psw(*pneu_states);
            if (detailed && kParityTicks[to_index(step % 3)].expect_contact) {
                require(psw[0].patch_count > 0 || psw[1].patch_count > 0,
                        "detailed pneumatic rollout populates contact patches");
            }
        }
        {
            const std::span<const GpuPneumaticWheelState<float>, 2> psw_final(*pneu_states);
            require(psw_final[0].carcass.initialized == 1 && psw_final[1].carcass.initialized == 1,
                    "pneumatic_wheels carcass state initialized");
        }
        require(max_radial_e > 0.0f && max_maxwell_e > 0.0f,
                "pneumatic rollout produces positive radial_energy_j and maxwell_energy_j");
        require(sum_radial_loss >= 0.0f && sum_brush_loss > 0.0f,
                "pneumatic rollout produces non-negative radial_loss_j and positive brush_and_rolling_loss_j");
        if (detailed) {
            require(max_shear_e > 0.0f,
                    "detailed pneumatic rollout produces positive shear_energy_j");
        }

        verify_metal_writers_parity(
            fx, bundle, terrain, dt_f, detailed ? "pneumatic_detailed" : "pneumatic_fast");
    }
}

constexpr std::array<TestCase, 8> cases{{
    {.name = "compliant_flat_parity", .run = test_compliant_flat_parity},
    {.name = "compliant_rough_uphill_parity", .run = test_compliant_rough_uphill_parity},
    {.name = "distributed_flat_and_rough_parity", .run = test_distributed_flat_and_rough_parity},
    {.name = "airborne_effective_radius_zero", .run = test_airborne_effective_radius_zero},
    {.name = "stiffness_scale_and_validation", .run = test_stiffness_scale_and_validation},
    {.name = "tabulated_compliant_parity", .run = test_tabulated_compliant_parity},
    {.name = "distributed_stage_a_transitions_parity", .run = test_distributed_stage_a_transitions_parity},
    {.name = "pneumatic_fast_and_detailed_parity", .run = test_pneumatic_fast_and_detailed_parity},
}};

int run_case(const TestCase &test_case, const fs::path &xml_path) {
    try {
        test_case.run(xml_path);
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
            std::cerr << "usage: bike_test_gpu_tire [case-name|--list]\n";
            return 2;
        }
        const fs::path xml_path = resolve_welded_xml("");
        require(!xml_path.empty(), "could not locate tests/fixtures/model/welded.xml");

        const std::string_view requested =
            arguments.size() == 1 ? "all" : std::string_view(arguments[1]);
        bool found = requested == "all";
        int status = 0;
        for (const TestCase &tc : cases) {
            if (requested == "all" || requested == tc.name) {
                found = true;
                if (run_case(tc, xml_path) != 0) {
                    status = 1;
                }
            }
        }
        if (!found) {
            std::cerr << "unknown gpu tire case: " << requested << '\n';
            return 2;
        }
        return status;
    } catch (const std::exception &error) {
        std::cerr << "FAIL argument handling: " << error.what() << '\n';
        return 1;
    }
}
