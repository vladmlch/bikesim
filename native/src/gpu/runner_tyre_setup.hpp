// native/src/gpu/runner_tyre_setup.hpp — Shared host-side helper for building
// default GpuTyreModel + terrain tables from mjModel when callers pass null
// overrides, or validating and copying explicit caller-supplied buffers.
#pragma once

#include "../model_access.hpp"
#include "../physics_core/tyre_upload.hpp"
#include "common.hpp"
#include "model_assets.hpp"
#include "terrain_tables.hpp"

#if defined(BIKE_GPU_STANDALONE_RUNNER)
#include "standalone_mujoco_shim.hpp"
#else
#include "../engine_call.hpp"
#endif

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

inline nativecfg::TireConfig make_default_distributed_tire_config(int station_count = 128) {
    nativecfg::TireParams side{};
    side.tangent_k_n_m = 20000.0;
    side.mu = 1.1;
    side.relaxation_length_m = 0.2;

    nativecfg::HingeDensity hd{};
    hd.knots_m = {0.0, 0.004, 0.012};
    hd.stiffness_n_m = {300000.0, 0.0, 0.0};
    hd.damping_ns_m = 0.0;
    hd.loss_factor = 0.05;
    hd.hysteresis_rate_eps_mps = 0.01;
    hd.provenance = "synthetic density; not fitted or measured";

    nativecfg::DistributedMaterial dmat{};
    dmat.density = hd;
    dmat.fitting_dataset_id = "none-synthetic";
    dmat.valid_load_range_n = {0.0, 2000.0};
    dmat.valid_deflection_range_m = {0.0, 0.04};
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
    cfg.significant_delta_m = 0.0001;
    cfg.significance_fraction = 0.05;
    cfg.distinct_normal_deg = 20.0;
    return cfg;
}

inline nativecfg::SurfaceSpec make_surface_spec_from_label(int surface_label) {
    // Mirrors native/src/terrain/surface.cpp registry (SurfaceLabel enum: 0..4).
    switch (surface_label) {
        case 0:
            return {.name = "asphalt", .mu_peak = 1.05, .mu_slide = 0.75,
                    .slip_stiffness_per_load = 15.0, .stribeck_speed_mps = 4.5,
                    .crr_delta = 0.0, .load_sensitivity = 0.06};
        case 1:
            return {.name = "hardpack", .mu_peak = 0.80, .mu_slide = 0.60,
                    .slip_stiffness_per_load = 12.0, .stribeck_speed_mps = 4.5,
                    .crr_delta = 0.0025, .load_sensitivity = 0.08};
        case 2:
            return {.name = "loose", .mu_peak = 0.55, .mu_slide = 0.45,
                    .slip_stiffness_per_load = 7.0, .stribeck_speed_mps = 4.5,
                    .crr_delta = 0.0060, .load_sensitivity = 0.12};
        case 3:
            return {.name = "wet", .mu_peak = 0.50, .mu_slide = 0.40,
                    .slip_stiffness_per_load = 10.0, .stribeck_speed_mps = 4.5,
                    .crr_delta = 0.0035, .load_sensitivity = 0.12};
        case 4:
            return {.name = "scree", .mu_peak = 0.45, .mu_slide = 0.38,
                    .slip_stiffness_per_load = 4.5, .stribeck_speed_mps = 4.5,
                    .crr_delta = 0.0080, .load_sensitivity = 0.15};
        default:
            return {.name = "hardpack", .mu_peak = 0.80, .mu_slide = 0.60,
                    .slip_stiffness_per_load = 12.0, .stribeck_speed_mps = 4.5,
                    .crr_delta = 0.0025, .load_sensitivity = 0.08};
    }
}

inline nativecfg::TireConfig make_default_pneumatic_tire_config(bool detailed = false) {
    nativecfg::TireParams side{};
    side.tangent_k_n_m = 60000.0;
    side.mu = 1.1;
    side.relaxation_length_m = 0.06;

    nativecfg::PneumaticTyreSpec front{};
    front.name = "Schwalbe Magic Mary 29x2.8";
    front.outer_radius_mm = 379.0;
    front.rim_radius_mm = 320.0;
    front.section_height_mm = 68.0;
    front.pressure_bar = 1.2;
    front.crr_reference = 0.011;
    front.casing_width_mm = 65.7;
    front.compressed_casing_mm = 6.0;
    front.area_factor = 0.509;
    front.carcass_stiffness_n_mm2 = 0.136;
    front.contact_length_factor = 0.851;
    front.rate_stiffening = 0.22;
    front.rate_relaxation_s = 0.2;
    front.loss_factor = 0.055;
    front.hysteresis_rate_eps_mps = 0.01;
    front.relaxation_length_mm = 60.0;
    front.rim_stiffness_n_mm2 = 28.0;
    front.tread_loss_crr = 0.00586;

    nativecfg::PneumaticTyreSpec rear{};
    rear.name = "Schwalbe Hans Dampf 27.5x2.8";
    rear.outer_radius_mm = 360.0;
    rear.rim_radius_mm = 300.0;
    rear.section_height_mm = 68.0;
    rear.pressure_bar = 1.2;
    rear.crr_reference = 0.0103;
    rear.casing_width_mm = 65.7;
    rear.compressed_casing_mm = 6.0;
    rear.area_factor = 0.522;
    rear.carcass_stiffness_n_mm2 = 0.140;
    rear.contact_length_factor = 0.827;
    rear.rate_stiffening = 0.22;
    rear.rate_relaxation_s = 0.2;
    rear.loss_factor = 0.055;
    rear.hysteresis_rate_eps_mps = 0.01;
    rear.relaxation_length_mm = 60.0;
    rear.rim_stiffness_n_mm2 = 28.0;
    rear.tread_loss_crr = 0.00481;

    nativecfg::PneumaticConfig pneu{};
    pneu.tier = detailed ? "detailed" : "fast";
    pneu.n_rays = detailed ? 256 : 64;
    pneu.half_angle_deg = 75.0;
    pneu.discretised_brush = detailed;
    pneu.timestep_s = detailed ? 0.00025 : 0.0005;
    pneu.surface = make_surface_spec_from_label(1);
    pneu.front = std::move(front);
    pneu.rear = std::move(rear);

    nativecfg::TireConfig cfg{};
    cfg.backend = "pneumatic";
    cfg.surface_mode = "configured";
    cfg.front = side;
    cfg.rear = std::move(side);
    cfg.significant_delta_m = 0.0001;
    cfg.significance_fraction = 0.05;
    cfg.distinct_normal_deg = 20.0;
    cfg.pneumatic = std::move(pneu);
    return cfg;
}

struct ViewerWeldedTireOptions {
    int backend_kind = 0;
    int station_count = 0;
    int surface_label = -1;
};

inline nativecfg::TireConfig make_viewer_welded_tire_config(
    const ViewerWeldedTireOptions &opts
) {
    nativecfg::TireConfig cfg = (opts.backend_kind == 0)
        ? make_default_compliant_tire_config()
        : ((opts.backend_kind == 2)
               ? make_default_pneumatic_tire_config(opts.station_count >= 256)
               : make_default_distributed_tire_config(opts.station_count));

    if (opts.backend_kind == 2) {
        if (opts.surface_label >= 0 && cfg.pneumatic.has_value()) {
            cfg.pneumatic->surface = make_surface_spec_from_label(opts.surface_label);
        }
        return cfg;
    }

    cfg.significant_delta_m = 0.0001;
    cfg.significance_fraction = 0.05;
    cfg.distinct_normal_deg = 20.0;

    const auto set_side = [](nativecfg::TireParams &side, double loss_factor) {
        side.tangent_k_n_m = 20000.0;
        side.mu = 1.1;
        side.relaxation_length_m = 0.2;
        if (side.material.has_value()) {
            side.material->radial_k_n_m = 130000.0;
            side.material->loss_factor = loss_factor;
            side.material->hysteresis_rate_eps_mps = 0.01;
            side.material->pressure_pa_gauge = 200000.0;
            side.material->provenance = "synthetic";
            side.material->valid_load_range_n = {0.0, 2000.0};
        }
    };
    set_side(cfg.front, 0.0662);
    set_side(cfg.rear, 0.0690);

    if (opts.surface_label >= 0) {
        cfg.surface_mode = "track";
        cfg.surface_map = nativecfg::SurfaceMap{
            .surface = make_surface_spec_from_label(opts.surface_label),
            .sections = {},
        };
    } else {
        cfg.surface_mode = "configured";
        cfg.surface_map.reset();
    }
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
                front_geom, rear_geom, idx.front_wheel_body, idx.rear_wheel_body, idx.frame_body);
        out.model = bundle.gpu_model;
        out.terrain = pack_upload_bundle_terrain(bundle);
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "[resolve_runner_tyre_and_terrain] " << e.what() << '\n';
        return -1;
    }
}

inline int build_runner_tyre_and_terrain_buffers_for_surface(
    const char *xml_path,
    int backend_kind,
    int station_count,
    int surface_label,
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
    if (backend_kind != 0 && backend_kind != 1 && backend_kind != 2) {
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
        mjData *d = engine::make_data(m);
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
        engine::forward(m, d);

        const GpuModelIndices idx =
            (d_idx != nullptr) ? *d_idx : assets::resolve_gpu_model_indices(m);
        const int front_geom = find_wheel_contact_geom(
            m, idx.front_wheel_body, "front_contact", "geom_front_contact");
        const int rear_geom = find_wheel_contact_geom(
            m, idx.rear_wheel_body, "rear_contact", "geom_rear_contact");
        if (front_geom < 0 || rear_geom < 0) {
            return -4;
        }

        const nativecfg::TireConfig cfg = (surface_label >= -1)
            ? make_viewer_welded_tire_config(ViewerWeldedTireOptions{
                  .backend_kind = backend_kind,
                  .station_count = station_count,
                  .surface_label = surface_label,
              })
            : ((backend_kind == 0)
                   ? make_default_compliant_tire_config()
                   : ((backend_kind == 2)
                          ? make_default_pneumatic_tire_config(station_count >= 256)
                          : make_default_distributed_tire_config(station_count)));
        const auto bundle = std::make_unique<bike::physics_core::HostTyreUploadBundle>(
            bike::physics_core::build_gpu_tyre_model(
                m, d, cfg, front_geom, rear_geom, idx.front_wheel_body, idx.rear_wheel_body, idx.frame_body));
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
    return build_runner_tyre_and_terrain_buffers_for_surface(
        xml_path, backend_kind, station_count, -2, d_idx,
        out_tyre_model, out_terrain_desc,
        out_terrain_f32, io_f32_count, out_terrain_i32, io_i32_count);
}

} // namespace bike::gpu
