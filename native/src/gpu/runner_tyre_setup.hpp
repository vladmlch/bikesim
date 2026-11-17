// native/src/gpu/runner_tyre_setup.hpp — Shared host-side helper for building
// default GpuTyreModel + terrain tables from mjModel when callers pass null
// overrides, or validating and copying explicit caller-supplied buffers.
#pragma once

#include "../model_access.hpp"
#include "../physics_core/tyre_upload.hpp"
#include "common.hpp"
#include "model_assets.hpp"
#include "terrain_tables.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <memory>
#include <span>
#include <utility>
#include <vector>

namespace bike::gpu {

inline nativecfg::TireConfig make_default_compliant_tire_config() {
    nativecfg::TireMaterial mat{};
    mat.radial_k_n_m = 110000.0;
    mat.loss_factor = 0.05;
    mat.hysteresis_rate_eps_mps = 0.01;
    mat.pressure_pa_gauge = 150000.0;
    mat.provenance = "runner_default";
    mat.valid_load_range_n = {0.0, 4000.0};

    nativecfg::TireParams side{};
    side.material = mat;
    side.tangent_k_n_m = 60000.0;
    side.mu = 1.1;
    side.relaxation_length_m = 0.05;

    nativecfg::TireConfig cfg{};
    cfg.backend = "compliant_2d";
    cfg.surface_mode = "configured";
    cfg.front = side;
    cfg.rear = std::move(side);
    cfg.significant_delta_m = 0.002;
    cfg.significance_fraction = 0.25;
    cfg.distinct_normal_deg = 12.0;
    return cfg;
}

inline nativecfg::TireConfig make_default_distributed_tire_config(int station_count = 64) {
    nativecfg::TireParams side{};
    side.tangent_k_n_m = 60000.0;
    side.mu = 1.1;
    side.relaxation_length_m = 0.05;

    nativecfg::HingeDensity hd{};
    hd.knots_m = {0.0, 0.01, 0.03, 0.06};
    hd.stiffness_n_m = {180000.0, 220000.0, 280000.0, 360000.0};
    hd.damping_ns_m = 0.0;
    hd.loss_factor = 0.05;
    hd.hysteresis_rate_eps_mps = 0.01;
    hd.provenance = "runner_default";

    nativecfg::DistributedMaterial dmat{};
    dmat.density = hd;
    dmat.fitting_dataset_id = "runner_default";
    dmat.valid_load_range_n = {0.0, 4000.0};
    dmat.valid_deflection_range_m = {0.0, 0.08};
    dmat.calibration_status = "experimental_unvalidated";

    nativecfg::DistributedConfig dcfg{};
    dcfg.station_count = station_count;
    dcfg.density = std::move(hd);
    dcfg.front = dmat;
    dcfg.rear = std::move(dmat);

    nativecfg::TireConfig cfg{};
    cfg.backend = "distributed_2d_reference";
    cfg.surface_mode = "configured";
    cfg.front = side;
    cfg.rear = std::move(side);
    cfg.distributed = dcfg;
    cfg.significant_delta_m = 0.002;
    cfg.significance_fraction = 0.25;
    cfg.distinct_normal_deg = 12.0;
    return cfg;
}

inline HostPackedTerrain pack_upload_bundle_terrain(
    const bike::physics_core::HostTyreUploadBundle &bundle
) {
    const HostPolylineSpans pl{
        .px = bundle.pl_px,
        .pz = bundle.pl_pz,
        .seg_x = bundle.pl_seg_x,
        .seg_z = bundle.pl_seg_z,
        .seg_len_sq = bundle.pl_seg_len_sq,
        .height_changes = bundle.pl_height_changes,
        .left_x = bundle.pl_left_x,
        .left_z = bundle.pl_left_z,
        .right_x = bundle.pl_right_x,
        .right_z = bundle.pl_right_z,
        .maximum_z = bundle.polyline_view.maximum_z,
        .significant_delta_m = bundle.polyline_view.significant_delta_m,
        .significance_fraction = bundle.polyline_view.significance_fraction,
        .normal_cosine = bundle.polyline_view.normal_cosine,
    };
    const HostSignedProfileSpans sp{
        .x = bundle.sp_x,
        .z = bundle.sp_z,
        .edge_x = bundle.sp_edge_x,
        .edge_z = bundle.sp_edge_z,
        .length2 = bundle.sp_length2,
        .normal_x = bundle.sp_normal_x,
        .normal_z = bundle.sp_normal_z,
    };
    return pack_terrain_tables(pl, sp);
}

inline int find_wheel_contact_geom(
    const mjModel *m,
    int wheel_body_id,
    const char *short_name,
    const char *long_name
) {
    if (m == nullptr || wheel_body_id <= 0 || wheel_body_id >= m->nbody) {
        return -1;
    }
    const std::span<const int> geom_bodyid =
        model_access::readonly_buffer(m->geom_bodyid, m->ngeom, "geom_bodyid");
    const int id_short = mj_name2id(m, mjOBJ_GEOM, short_name);
    if (id_short >= 0 && geom_bodyid[static_cast<std::size_t>(id_short)] == wheel_body_id) {
        return id_short;
    }
    const int id_long = mj_name2id(m, mjOBJ_GEOM, long_name);
    if (id_long >= 0 && geom_bodyid[static_cast<std::size_t>(id_long)] == wheel_body_id) {
        return id_long;
    }
    const std::span<const int> geom_type =
        model_access::readonly_buffer(m->geom_type, m->ngeom, "geom_type");
    const std::span<const mjtNum> geom_pos =
        model_access::readonly_buffer(m->geom_pos, model_access::kXYZ * m->ngeom, "geom_pos");
    for (int g = 0; g < m->ngeom; ++g) {
        const auto gu = static_cast<std::size_t>(g);
        if (geom_bodyid[gu] == wheel_body_id && geom_type[gu] == mjGEOM_SPHERE &&
            std::abs(geom_pos[3U * gu + 0U]) <= 1e-12 &&
            std::abs(geom_pos[3U * gu + 1U]) <= 1e-12 &&
            std::abs(geom_pos[3U * gu + 2U]) <= 1e-12) {
            return g;
        }
    }
    return -1;
}

struct ResolvedRunnerTyre {
    bike::physics_core::GpuTyreModel model{};
    HostPackedTerrain terrain{};
};

inline int resolve_runner_tyre_and_terrain(
    const mjModel *m,
    const mjData *d,
    const GpuModelIndices &idx,
    const bike::physics_core::GpuTyreModel *caller_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *caller_terrain_desc,
    std::span<const float> caller_terrain_f32,
    std::span<const bike::physics_core::i32> caller_terrain_i32,
    ResolvedRunnerTyre &out
) noexcept {
    if (m == nullptr || d == nullptr) {
        return -1;
    }
    if ((caller_tyre_model == nullptr) != (caller_terrain_desc == nullptr)) {
        std::cerr << "[resolve_runner_tyre_and_terrain] tyre_model and terrain_desc must both be provided or both null\n";
        return -1;
    }
    if (caller_tyre_model != nullptr) {
        try {
            if (validate_host_tyre_and_terrain(
                    caller_tyre_model,
                    idx.nq,
                    idx.nv,
                    idx.nbody,
                    caller_terrain_desc,
                    caller_terrain_f32.empty() ? nullptr : caller_terrain_f32.data(),
                    caller_terrain_i32.empty() ? nullptr : caller_terrain_i32.data()) != 0) {
                return -1;
            }
            out.model = *caller_tyre_model;
            out.terrain.desc = *caller_terrain_desc;
            out.terrain.f32.assign(caller_terrain_f32.begin(), caller_terrain_f32.end());
            out.terrain.i32.assign(caller_terrain_i32.begin(), caller_terrain_i32.end());
            return 0;
        } catch (...) {
            return -1;
        }
    }

    try {
        const int front_geom = find_wheel_contact_geom(
            m, idx.front_wheel_body, "front_contact", "geom_front_contact");
        const int rear_geom = find_wheel_contact_geom(
            m, idx.rear_wheel_body, "rear_contact", "geom_rear_contact");
        if (front_geom < 0 || rear_geom < 0) {
            std::cerr << "[resolve_runner_tyre_and_terrain] Failed to find front/rear wheel contact geoms\n";
            return -1;
        }
        const bike::physics_core::HostTyreUploadBundle bundle =
            bike::physics_core::build_gpu_tyre_model(
                m, d, make_default_compliant_tire_config(),
                front_geom, rear_geom, idx.front_wheel_body, idx.rear_wheel_body);
        out.model = bundle.gpu_model;
        out.terrain = pack_upload_bundle_terrain(bundle);
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "[resolve_runner_tyre_and_terrain] " << e.what() << '\n';
        return -1;
    }
}

inline int build_runner_tyre_and_terrain_buffers(
    const char *xml_path,
    int backend_kind,
    int station_count,
    const GpuModelIndices *d_idx,
    bike::physics_core::GpuTyreModel *out_tyre_model,
    bike::physics_core::GpuTerrainTableDesc *out_terrain_desc,
    float *out_terrain_f32,
    int *io_f32_count,
    bike::physics_core::i32 *out_terrain_i32,
    int *io_i32_count
) noexcept {
    if (xml_path == nullptr || out_tyre_model == nullptr || out_terrain_desc == nullptr ||
        io_f32_count == nullptr || io_i32_count == nullptr) {
        return -1;
    }
    if (backend_kind != 0 && backend_kind != 1) {
        return -1;
    }
    if (backend_kind == 1 &&
        (station_count <= 0 || station_count > bike::physics_core::kMaxGpuTyreStations)) {
        return -1;
    }

    try {
        std::array<char, 1024> err{};
        mjModel *m = mj_loadXML(xml_path, nullptr, err.data(), static_cast<int>(err.size()));
        if (m == nullptr) {
            return -2;
        }
        const auto m_guard = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>(m, &mj_deleteModel);
        if (assets::load_hfield_bin(m, xml_path) != assets::AssetStatus::kOk) {
            return -3;
        }
        mjData *d = mj_makeData(m);
        if (d == nullptr) {
            return -2;
        }
        const auto d_guard = std::unique_ptr<mjData, decltype(&mj_deleteData)>(d, &mj_deleteData);

        std::vector<double> qp;
        std::vector<double> qv;
        if (assets::load_captured_initial_state(
                xml_path, static_cast<int>(m->nq), static_cast<int>(m->nv), qp, qv) ==
            assets::AssetStatus::kOk) {
            const std::span<mjtNum> d_qpos = model_access::mutable_buffer(d->qpos, m->nq, "qpos");
            const std::span<mjtNum> d_qvel = model_access::mutable_buffer(d->qvel, m->nv, "qvel");
            for (std::size_t i = 0; i < qp.size(); ++i) {
                d_qpos[i] = static_cast<mjtNum>(qp[i]);
            }
            for (std::size_t i = 0; i < qv.size(); ++i) {
                d_qvel[i] = static_cast<mjtNum>(qv[i]);
            }
        }
        mj_forward(m, d);

        const GpuModelIndices idx =
            (d_idx != nullptr) ? *d_idx : assets::resolve_gpu_model_indices(m);
        const int front_geom = find_wheel_contact_geom(
            m, idx.front_wheel_body, "front_contact", "geom_front_contact");
        const int rear_geom = find_wheel_contact_geom(
            m, idx.rear_wheel_body, "rear_contact", "geom_rear_contact");
        if (front_geom < 0 || rear_geom < 0) {
            return -4;
        }

        const nativecfg::TireConfig cfg = (backend_kind == 0)
                                              ? make_default_compliant_tire_config()
                                              : make_default_distributed_tire_config(station_count);
        const auto bundle = std::make_unique<bike::physics_core::HostTyreUploadBundle>(
            bike::physics_core::build_gpu_tyre_model(
                m, d, cfg, front_geom, rear_geom, idx.front_wheel_body, idx.rear_wheel_body));
        const HostPackedTerrain terrain = pack_upload_bundle_terrain(*bundle);

        const int req_f32 = static_cast<int>(terrain.f32.size());
        const int req_i32 = static_cast<int>(terrain.i32.size());
        *out_tyre_model = bundle->gpu_model;
        *out_terrain_desc = terrain.desc;

        if (out_terrain_f32 == nullptr && out_terrain_i32 == nullptr) {
            *io_f32_count = req_f32;
            *io_i32_count = req_i32;
            return 0;
        }

        if (*io_f32_count < req_f32 || *io_i32_count < req_i32 ||
            (req_f32 > 0 && out_terrain_f32 == nullptr) ||
            (req_i32 > 0 && out_terrain_i32 == nullptr)) {
            *io_f32_count = req_f32;
            *io_i32_count = req_i32;
            return -5;
        }

        if (req_f32 > 0) {
            std::ranges::copy(terrain.f32, out_terrain_f32);
        }
        if (req_i32 > 0) {
            std::ranges::copy(terrain.i32, out_terrain_i32);
        }
        *io_f32_count = req_f32;
        *io_i32_count = req_i32;
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "[build_runner_tyre_and_terrain_buffers] " << e.what() << '\n';
        return -1;
    }
}

} // namespace bike::gpu
