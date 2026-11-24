// native/src/physics_core/tyre_gpu_step.hpp — Single-source tri-target (Host C++,
// CUDA, Apple Metal MSL) wheel-step binding unifying terrain_scanner,
// terrain_distance, kinematics, tyre_compliant, and tyre_distributed over flat
// POD terrain descriptor + device tables.
#pragma once

#include "common.hpp"
#include "gpu_math.hpp"
#include "gpu_toolkit.hpp"
#include "kinematics.hpp"
#include "terrain_distance.hpp"
#include "terrain_scanner.hpp"
#include "tyre_compliant.hpp"
#include "tyre_contact.hpp"
#include "tyre_distributed.hpp"
#include "tyre_pneumatic_model.hpp"
#include "tyre_tabulated.hpp"
#include "tyre_types.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: templated pointer parameters accept device/constant/thread buffers in MSL without address-space cast errors. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

static_assert(kMaxGpuRadialTableKnots == kMaxRadialTableKnots, "tabulated knot bound mismatch");
static_assert(kMaxGpuPneumaticRays == kMaxPneumaticRays, "pneumatic ray bound mismatch");
static_assert(kMaxGpuPneumaticPatches == kMaxPneumaticPatches, "pneumatic patch bound mismatch");

struct GpuTerrainTableDesc {
    i32 polyline_vertex_count = 0;
    i32 px_offset = 0;
    i32 pz_offset = 0;
    i32 seg_x_offset = 0;
    i32 seg_z_offset = 0;
    i32 seg_len_sq_offset = 0;
    i32 height_changes_offset = 0;
    i32 left_x_offset = 0;
    i32 left_z_offset = 0;
    i32 right_x_offset = 0;
    i32 right_z_offset = 0;
    float maximum_z = 0.0f;
    float significant_delta_m = 0.0f;
    float significance_fraction = 0.0f;
    float normal_cosine = 0.0f;

    i32 signed_vertex_count = 0;
    i32 sp_x_offset = 0;
    i32 sp_z_offset = 0;
    i32 sp_edge_x_offset = 0;
    i32 sp_edge_z_offset = 0;
    i32 sp_length2_offset = 0;
    i32 sp_normal_x_offset = 0;
    i32 sp_normal_z_offset = 0;

    i32 f32_count = 0;
    i32 i32_count = 0;
};

struct GpuWheelSummary {
    float normal_load_n = 0.0f;
    float effective_radius_m = 0.0f;
    float slip_mps = 0.0f;
    float couple_world_nm = 0.0f;
    float rolling_dissipation_j = 0.0f;
    float radial_energy_j = 0.0f;
    float shear_energy_j = 0.0f;
    float maxwell_energy_j = 0.0f;
    float rim_energy_j = 0.0f;
    float radial_loss_j = 0.0f;
    float brush_and_rolling_loss_j = 0.0f;
    float rolling_moment_nm = 0.0f;
};
using GpuWheelStepSummary = GpuWheelSummary;

struct GpuTyreStepOutput {
    GpuWheelSummary wheels[2]{};
    i32 active_station_counts[2]{};
    float radial_energy_j = 0.0f;
    float shear_energy_j = 0.0f;
    float maxwell_energy_j = 0.0f;
    float rim_energy_j = 0.0f;
    float radial_loss_j = 0.0f;
    float brush_and_rolling_loss_j = 0.0f;
    float rolling_moment_nm = 0.0f;
};

struct ScaledCompliantView {
    float radial_k_n_m = 0.0f;
    float loss_factor = 0.0f;
    float hysteresis_rate_eps_mps = 0.0f;
    float valid_load_min_n = 0.0f;
    float valid_load_max_n = 0.0f;
    u8 has_tabulated_radial = 0;
    u8 use_tabulated = 0;
    u8 pad[2]{};
};

struct ScaledTyreWheelSpec {
    float radius_m = 0.0f;
    float tangent_k_n_m = 0.0f;
    float tire_mu = 0.0f;
    float relaxation_length_m = 0.0f;
    ScaledCompliantView compliant{};
    HingeDensityView<float> distributed_density{};
    float station_weight = 0.0f;
};

struct ScaledTyreModel {
    i32 nv = 0;
    i32 station_count = 0;
    float distinct_normal_cosine = 0.0f;
    ScaledTyreWheelSpec wheels[2]{};
    SurfaceMapView<float> surface_map{};
};

struct GpuPneumaticRingRef {
    i32 n_rays = 0;
    float half_angle_rad = 0.0f;
    float dtheta_rad = 0.0f;
    BIKE_DEVICE const float *theta_rad = nullptr;
    BIKE_DEVICE const float *ux = nullptr;
    BIKE_DEVICE const float *uz = nullptr;
};

struct GpuPneumaticSpecRef {
    PneumaticTyreSpecPod<float> tyre{};
    GpuPneumaticRingRef ring{};
    GpuWheelChain chain{};
    i32 wheel_body_id = -1;
    i32 nv = 0;
    i32 njnt = 0;
    u8 discretised_brush = 0;
    u8 pad[3]{};
};

BIKE_HD bool terrain_span_in_bounds(i32 offset, i32 len, i32 total) noexcept {
    return offset >= 0 && len >= 0 && total >= 0 && offset <= total && len <= (total - offset);
}

BIKE_HD Status validate_terrain_table_desc(
    BIKE_CREF(GpuTerrainTableDesc) desc,
    BIKE_DEVICE const float *terrain_f32,
    BIKE_DEVICE const i32 *terrain_i32,
    bool require_polyline,
    bool require_signed,
    bool check_monotonic,
    BIKE_REF(ErrCode) err
) noexcept {
    err = ErrCode::none;
    if (desc.f32_count < 0 || desc.i32_count < 0) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (require_polyline && desc.polyline_vertex_count < 2) {
        err = ErrCode::degenerate_contact;
        return Status::invalid_argument;
    }
    if (require_signed && desc.signed_vertex_count < 2) {
        err = ErrCode::degenerate_contact;
        return Status::invalid_argument;
    }

    if (desc.polyline_vertex_count > 0) {
        if (desc.polyline_vertex_count < 2 || terrain_f32 == nullptr || terrain_i32 == nullptr) {
            err = ErrCode::degenerate_contact;
            return Status::invalid_argument;
        }
        const i32 n = desc.polyline_vertex_count;
        const i32 s = n - 1;
        if (!terrain_span_in_bounds(desc.px_offset, n, desc.f32_count) ||
            !terrain_span_in_bounds(desc.pz_offset, n, desc.f32_count) ||
            !terrain_span_in_bounds(desc.seg_x_offset, s, desc.f32_count) ||
            !terrain_span_in_bounds(desc.seg_z_offset, s, desc.f32_count) ||
            !terrain_span_in_bounds(desc.seg_len_sq_offset, s, desc.f32_count) ||
            !terrain_span_in_bounds(desc.left_x_offset, n, desc.f32_count) ||
            !terrain_span_in_bounds(desc.left_z_offset, n, desc.f32_count) ||
            !terrain_span_in_bounds(desc.right_x_offset, n, desc.f32_count) ||
            !terrain_span_in_bounds(desc.right_z_offset, n, desc.f32_count) ||
            !terrain_span_in_bounds(desc.height_changes_offset, n, desc.i32_count)) {
            err = ErrCode::outside_domain;
            return Status::out_of_bounds;
        }
        if (!GpuToolkit::isfinite(desc.maximum_z) ||
            !GpuToolkit::isfinite(desc.significant_delta_m) || desc.significant_delta_m < 0.0f ||
            !GpuToolkit::isfinite(desc.significance_fraction) || desc.significance_fraction < 0.0f ||
            desc.significance_fraction > 1.0f || !GpuToolkit::isfinite(desc.normal_cosine)) {
            err = ErrCode::degenerate_contact;
            return Status::invalid_argument;
        }
        if (check_monotonic) {
            BIKE_DEVICE const float *px = terrain_f32 + desc.px_offset;
            BIKE_DEVICE const float *pz = terrain_f32 + desc.pz_offset;
            if (!GpuToolkit::isfinite(px[0]) || !GpuToolkit::isfinite(pz[0])) {
                err = ErrCode::degenerate_contact;
                return Status::invalid_argument;
            }
            for (i32 i = 0; i < s; ++i) {
                if (!GpuToolkit::isfinite(px[i + 1]) || !GpuToolkit::isfinite(pz[i + 1]) ||
                    !(px[i + 1] > px[i])) {
                    err = ErrCode::degenerate_contact;
                    return Status::invalid_argument;
                }
            }
        }
    }

    if (desc.signed_vertex_count > 0) {
        if (desc.signed_vertex_count < 2 || terrain_f32 == nullptr) {
            err = ErrCode::degenerate_contact;
            return Status::invalid_argument;
        }
        const i32 n = desc.signed_vertex_count;
        const i32 s = n - 1;
        if (!terrain_span_in_bounds(desc.sp_x_offset, n, desc.f32_count) ||
            !terrain_span_in_bounds(desc.sp_z_offset, n, desc.f32_count) ||
            !terrain_span_in_bounds(desc.sp_edge_x_offset, s, desc.f32_count) ||
            !terrain_span_in_bounds(desc.sp_edge_z_offset, s, desc.f32_count) ||
            !terrain_span_in_bounds(desc.sp_length2_offset, s, desc.f32_count) ||
            !terrain_span_in_bounds(desc.sp_normal_x_offset, s, desc.f32_count) ||
            !terrain_span_in_bounds(desc.sp_normal_z_offset, s, desc.f32_count)) {
            err = ErrCode::outside_domain;
            return Status::out_of_bounds;
        }
        if (check_monotonic) {
            BIKE_DEVICE const float *sx = terrain_f32 + desc.sp_x_offset;
            BIKE_DEVICE const float *sz = terrain_f32 + desc.sp_z_offset;
            if (!GpuToolkit::isfinite(sx[0]) || !GpuToolkit::isfinite(sz[0])) {
                err = ErrCode::degenerate_contact;
                return Status::invalid_argument;
            }
            for (i32 i = 0; i < s; ++i) {
                if (!GpuToolkit::isfinite(sx[i + 1]) || !GpuToolkit::isfinite(sz[i + 1]) ||
                    !(sx[i + 1] > sx[i])) {
                    err = ErrCode::degenerate_contact;
                    return Status::invalid_argument;
                }
            }
        }
    }

    return Status::ok;
}

template <class ModelPtr>
BIKE_HD Status validate_gpu_tyre_model_device(
    ModelPtr model,
    i32 expected_nq,
    i32 expected_nv,
    i32 expected_nbody,
    BIKE_REF(ErrCode) err
) noexcept {
    err = ErrCode::none;
    if (model == nullptr) {
        err = ErrCode::missing_material;
        return Status::invalid_argument;
    }
    if (model->backend != TyreBackendKind::compliant_2d &&
        model->backend != TyreBackendKind::distributed_2d_reference &&
        model->backend != TyreBackendKind::pneumatic) {
        err = ErrCode::missing_material;
        return Status::invalid_argument;
    }
    if (model->nq <= 0 || model->nq > static_cast<i32>(kMaxNq) ||
        model->nv <= 0 || model->nv > static_cast<i32>(kMaxNv) ||
        model->nbody <= 0 || model->nbody > static_cast<i32>(kMaxNbody) ||
        model->njnt <= 0 || model->njnt > static_cast<i32>(kMaxNjnt)) {
        err = ErrCode::out_of_bounds_chain;
        return Status::out_of_bounds;
    }
    if ((expected_nq > 0 && model->nq != expected_nq) ||
        (expected_nv > 0 && model->nv != expected_nv) ||
        (expected_nbody > 0 && model->nbody != expected_nbody)) {
        err = ErrCode::out_of_bounds_chain;
        return Status::invalid_argument;
    }
    if (model->surface_map.section_count < 0 ||
        model->surface_map.section_count > static_cast<i32>(kMaxSurfaceSections)) {
        err = ErrCode::missing_material;
        return Status::out_of_bounds;
    }
    if (model->backend == TyreBackendKind::distributed_2d_reference) {
        if (model->station_count <= 0 ||
            model->station_count > static_cast<i32>(kMaxGpuTyreStations)) {
            err = ErrCode::out_of_bounds_station;
            return Status::out_of_bounds;
        }
    }

    for (i32 side = 0; side < 2; ++side) {
        const float radius = model->wheels[side].radius_m;
        const float tk = model->wheels[side].tangent_k_n_m;
        const float mu = model->wheels[side].tire_mu;
        const float relax = model->wheels[side].relaxation_length_m;
        if (!GpuToolkit::isfinite(radius) || radius <= 0.0f ||
            !GpuToolkit::isfinite(tk) || tk <= 0.0f ||
            !GpuToolkit::isfinite(mu) || mu < 0.0f ||
            !GpuToolkit::isfinite(relax) || relax <= 0.0f) {
            err = ErrCode::invalid_radius;
            return Status::invalid_argument;
        }
        if (model->backend == TyreBackendKind::compliant_2d) {
            if (model->wheels[side].compliant.use_tabulated != 0 ||
                model->wheels[side].compliant.has_tabulated_radial != 0) {
                const i32 tk_cnt = model->wheels[side].compliant.knot_count;
                if (model->wheels[side].compliant.use_tabulated == 0 ||
                    model->wheels[side].compliant.has_tabulated_radial == 0 ||
                    tk_cnt < 2 || tk_cnt > kMaxGpuRadialTableKnots) {
                    err = ErrCode::missing_material;
                    return Status::invalid_argument;
                }
            } else if (!GpuToolkit::isfinite(model->wheels[side].compliant.radial_k_n_m) ||
                       model->wheels[side].compliant.radial_k_n_m < 0.0f) {
                err = ErrCode::missing_material;
                return Status::invalid_argument;
            }
        } else if (model->backend == TyreBackendKind::distributed_2d_reference) {
            const i32 kcnt = model->wheels[side].distributed_density.knot_count;
            if (kcnt < 2 || kcnt > static_cast<i32>(kMaxHingeKnots)) {
                err = ErrCode::missing_material;
                return Status::out_of_bounds;
            }
        } else {
            const i32 nrays = model->wheels[side].pneumatic.n_rays;
            if (nrays < 3 || nrays > kMaxGpuPneumaticRays ||
                !GpuToolkit::isfinite(model->wheels[side].pneumatic.outer_radius_mm) ||
                model->wheels[side].pneumatic.outer_radius_mm <= 0.0f ||
                !GpuToolkit::isfinite(model->wheels[side].pneumatic.rim_radius_mm) ||
                model->wheels[side].pneumatic.rim_radius_mm <= 0.0f ||
                model->wheels[side].pneumatic.rim_radius_mm >=
                    model->wheels[side].pneumatic.outer_radius_mm ||
                !GpuToolkit::isfinite(model->wheels[side].pneumatic.dtheta_rad) ||
                model->wheels[side].pneumatic.dtheta_rad <= 0.0f) {
                err = ErrCode::missing_material;
                return Status::invalid_argument;
            }
            if (model->road_is_uniform_grid != 0 &&
                (!GpuToolkit::isfinite(model->road_x0_m) ||
                 !GpuToolkit::isfinite(model->road_dx_m) ||
                 model->road_dx_m <= 0.0f)) {
                err = ErrCode::outside_domain;
                return Status::invalid_argument;
            }
        }
        const i32 body_id = model->wheels[side].chain.wheel_body_id;
        const i32 njnt = model->wheels[side].chain.njnt;
        const i32 jcnt = model->wheels[side].chain.joint_count;
        if (body_id <= 0 || body_id >= model->nbody ||
            njnt != model->njnt ||
            jcnt < 0 || jcnt > static_cast<i32>(kMaxChainJoints)) {
            err = ErrCode::out_of_bounds_chain;
            return Status::out_of_bounds;
        }
        for (i32 k = 0; k < jcnt; ++k) {
            const GpuChainJoint j = model->wheels[side].chain.joints[k];
            if (j.joint_id < 0 || j.joint_id >= model->njnt ||
                j.dof_adr < 0 || j.dof_adr >= model->nv) {
                err = ErrCode::out_of_bounds_chain;
                return Status::out_of_bounds;
            }
            if (j.joint_type != kJntHinge && j.joint_type != kJntSlide) {
                err = ErrCode::out_of_bounds_chain;
                return Status::invalid_argument;
            }
        }
    }
    {
        const i32 f_body_id = model->frame_chain.wheel_body_id;
        const i32 f_njnt = model->frame_chain.njnt;
        const i32 f_jcnt = model->frame_chain.joint_count;
        if (f_body_id <= 0 || f_body_id >= model->nbody ||
            f_njnt != model->njnt ||
            f_jcnt < 0 || f_jcnt > static_cast<i32>(kMaxChainJoints)) {
            err = ErrCode::out_of_bounds_chain;
            return Status::out_of_bounds;
        }
        for (i32 k = 0; k < f_jcnt; ++k) {
            const GpuChainJoint j = model->frame_chain.joints[k];
            if (j.joint_id < 0 || j.joint_id >= model->njnt ||
                j.dof_adr < 0 || j.dof_adr >= model->nv) {
                err = ErrCode::out_of_bounds_chain;
                return Status::out_of_bounds;
            }
            if (j.joint_type != kJntHinge && j.joint_type != kJntSlide) {
                err = ErrCode::out_of_bounds_chain;
                return Status::invalid_argument;
            }
        }
    }
    return Status::ok;
}

BIKE_HD PolylineView<float> make_polyline_view(
    BIKE_CREF(GpuTerrainTableDesc) desc,
    BIKE_DEVICE const float *terrain_f32,
    BIKE_DEVICE const i32 *terrain_i32
) noexcept {
    PolylineView<float> pl{};
    pl.px = terrain_f32 + desc.px_offset;
    pl.pz = terrain_f32 + desc.pz_offset;
    pl.seg_x = terrain_f32 + desc.seg_x_offset;
    pl.seg_z = terrain_f32 + desc.seg_z_offset;
    pl.seg_len_sq = terrain_f32 + desc.seg_len_sq_offset;
    pl.height_changes =
        (terrain_i32 != nullptr) ? (terrain_i32 + desc.height_changes_offset) : nullptr;
    pl.left_x = terrain_f32 + desc.left_x_offset;
    pl.left_z = terrain_f32 + desc.left_z_offset;
    pl.right_x = terrain_f32 + desc.right_x_offset;
    pl.right_z = terrain_f32 + desc.right_z_offset;
    pl.num_vertices = desc.polyline_vertex_count;
    pl.maximum_z = desc.maximum_z;
    pl.significant_delta_m = desc.significant_delta_m;
    pl.significance_fraction = desc.significance_fraction;
    pl.normal_cosine = desc.normal_cosine;
    return pl;
}

BIKE_HD SignedProfileView<float> make_signed_profile_view(
    BIKE_CREF(GpuTerrainTableDesc) desc,
    BIKE_DEVICE const float *terrain_f32
) noexcept {
    SignedProfileView<float> sp{};
    sp.x = terrain_f32 + desc.sp_x_offset;
    sp.z = terrain_f32 + desc.sp_z_offset;
    sp.edge_x = terrain_f32 + desc.sp_edge_x_offset;
    sp.edge_z = terrain_f32 + desc.sp_edge_z_offset;
    sp.length2 = terrain_f32 + desc.sp_length2_offset;
    sp.normal_x = terrain_f32 + desc.sp_normal_x_offset;
    sp.normal_z = terrain_f32 + desc.sp_normal_z_offset;
    sp.num_vertices = desc.signed_vertex_count;
    return sp;
}

template <class ModelPtr>
BIKE_HD ScaledTyreModel make_scaled_tyre_model(
    ModelPtr model,
    float tire_stiff_scale
) noexcept {
    const float scale = GpuToolkit::max(0.0f, tire_stiff_scale);
    ScaledTyreModel out{};
    out.nv = model->nv;
    out.station_count = model->station_count;
    out.distinct_normal_cosine = model->distinct_normal_cosine;
    out.surface_map = model->surface_map;
    for (i32 side = 0; side < 2; ++side) {
        out.wheels[side].radius_m = model->wheels[side].radius_m;
        out.wheels[side].tangent_k_n_m = model->wheels[side].tangent_k_n_m * scale;
        out.wheels[side].tire_mu = model->wheels[side].tire_mu;
        out.wheels[side].relaxation_length_m = model->wheels[side].relaxation_length_m;
        out.wheels[side].compliant.radial_k_n_m = model->wheels[side].compliant.radial_k_n_m * scale;
        out.wheels[side].compliant.loss_factor = model->wheels[side].compliant.loss_factor;
        out.wheels[side].compliant.hysteresis_rate_eps_mps =
            model->wheels[side].compliant.hysteresis_rate_eps_mps;
        out.wheels[side].compliant.valid_load_min_n =
            model->wheels[side].compliant.valid_load_min_n;
        out.wheels[side].compliant.valid_load_max_n =
            model->wheels[side].compliant.valid_load_max_n;
        out.wheels[side].compliant.has_tabulated_radial =
            model->wheels[side].compliant.has_tabulated_radial;
        out.wheels[side].compliant.use_tabulated =
            model->wheels[side].compliant.use_tabulated;
        out.wheels[side].distributed_density = model->wheels[side].distributed_density;
        const i32 kcnt = out.wheels[side].distributed_density.knot_count;
        for (i32 k = 0; k < kcnt && k < static_cast<i32>(kMaxHingeKnots); ++k) {
            out.wheels[side].distributed_density.stiffness_n_m[k] *= scale;
        }
        out.wheels[side].station_weight = model->wheels[side].station_weight;
    }
    return out;
}

struct GpuWheelSummaryIo {
    BIKE_DEVICE const GpuTyreModel *model = nullptr;
    float tire_stiff_scale = 1.0f;
    float dt_s = 0.0f;
    float tk_scaled[2]{};
    float nominal_radius[2]{};
    TyreVec3<float> wheel_center[2]{};
    float sum_load[2]{};
    float sum_weighted_drop[2]{};
    float sum_unweighted_drop[2]{};
    float sum_weighted_slip[2]{};
    float sum_elastic_energy[2]{};
    GpuTyreStepOutput summary{};

    BIKE_HD bool eval_radial(
        i32 side,
        BIKE_CREF(RadialEvalInput<float>) in,
        BIKE_REF(RadialEvalOutput<float>) out,
        BIKE_REF(Status) status,
        BIKE_REF(ErrCode) err
    ) noexcept {
        if (model == nullptr || side < 0 || side > 1 ||
            model->wheels[side].compliant.use_tabulated == 0) {
            return false;
        }
        BIKE_DEVICE const CompliantMaterialView<float> *c =
            &model->wheels[side].compliant;
        float force = 0.0f;
        float energy = 0.0f;
        status = tabulated_radial_contact_arrays<float, GpuToolkit>(
            c->knot_count, c->deflection_m, c->force_n, c->slopes, c->integral,
            in.delta, in.delta_dot, in.eta, in.eps,
            force, energy, err);
        if (status != Status::ok) {
            return true;
        }
        float elastic = 0.0f;
        float unused_energy = 0.0f;
        status = tabulated_radial_elastic_arrays<float, GpuToolkit>(
            c->knot_count, c->deflection_m, c->force_n, c->slopes, c->integral,
            in.delta, elastic, unused_energy, err);
        if (status != Status::ok) {
            return true;
        }
        const float scale = GpuToolkit::max(0.0f, tire_stiff_scale);
        out.normal = force * scale;
        out.energy = energy * scale;
        out.elastic = elastic * scale;
        return true;
    }

    BIKE_HD bool custom_friction(
        BIKE_CREF(FrictionQuery<float>),
        BIKE_REF(FrictionResult<float>),
        BIKE_REF(Status),
        BIKE_REF(ErrCode)
    ) noexcept {
        return false;
    }

    BIKE_HD Status record(
        i32 side,
        BIKE_CREF(CompliantRecord<float>) rec,
        BIKE_REF(ErrCode) err
    ) noexcept {
        err = ErrCode::none;
        summary.wheels[side].normal_load_n = rec.normal_load;
        const float delta = rec.contact.delta;
        if (delta > 0.0f) {
            const float dx = rec.point.x - rec.center.x;
            const float dy = rec.point.y - rec.center.y;
            const float dz = rec.point.z - rec.center.z;
            const float drop =
                -(dx * rec.normal.x + dy * rec.normal.y + dz * rec.normal.z);
            summary.wheels[side].effective_radius_m = GpuToolkit::max(0.0f, drop);
            summary.wheels[side].slip_mps = rec.slip;
            summary.active_station_counts[side] = 1;
        } else {
            summary.wheels[side].effective_radius_m = 0.0f;
            summary.wheels[side].slip_mps = 0.0f;
            summary.active_station_counts[side] = 0;
        }
        summary.wheels[side].radial_energy_j = rec.radial_energy;
        summary.wheels[side].shear_energy_j =
            0.5f * tk_scaled[side] * rec.xi_new * rec.xi_new;
        summary.wheels[side].maxwell_energy_j = 0.0f;
        summary.wheels[side].rim_energy_j = 0.0f;
        const float radial_power =
            delta > 0.0f
                ? GpuToolkit::max(
                      0.0f, (rec.normal_load - rec.elastic_force) * rec.delta_dot)
                : 0.0f;
        summary.wheels[side].radial_loss_j = radial_power * dt_s;
        summary.wheels[side].brush_and_rolling_loss_j =
            GpuToolkit::max(rec.release_loss, 0.0f) + rec.brush_loss;
        summary.wheels[side].rolling_moment_nm = 0.0f;
        return Status::ok;
    }

    BIKE_HD Status record_station(
        i32 side,
        BIKE_CREF(DistributedStationRecord<float>) rec,
        BIKE_REF(ErrCode) err
    ) noexcept {
        err = ErrCode::none;
        const float dx = rec.point.x - wheel_center[side].x;
        const float dy = rec.point.y - wheel_center[side].y;
        const float dz = rec.point.z - wheel_center[side].z;
        const float drop =
            -(dx * rec.normal.x + dy * rec.normal.y + dz * rec.normal.z);
        sum_load[side] += rec.load;
        sum_weighted_drop[side] += rec.load * drop;
        sum_unweighted_drop[side] += drop;
        sum_weighted_slip[side] += rec.load * rec.slip;
        sum_elastic_energy[side] += rec.elastic_energy;
        return Status::ok;
    }

    BIKE_HD Status record_wheel(
        i32 side,
        BIKE_CREF(DistributedWheelRecord<float>) rec,
        BIKE_REF(ErrCode) err
    ) noexcept {
        err = ErrCode::none;
        summary.active_station_counts[side] = rec.active_count;
        summary.wheels[side].normal_load_n = sum_load[side];
        if (rec.active_count <= 0) {
            summary.wheels[side].effective_radius_m = 0.0f;
            summary.wheels[side].slip_mps = 0.0f;
        } else if (sum_load[side] > 0.0f) {
            summary.wheels[side].effective_radius_m =
                GpuToolkit::max(0.0f, sum_weighted_drop[side] / sum_load[side]);
            summary.wheels[side].slip_mps = sum_weighted_slip[side] / sum_load[side];
        } else {
            summary.wheels[side].effective_radius_m = GpuToolkit::max(
                0.0f, sum_unweighted_drop[side] / static_cast<float>(rec.active_count));
            summary.wheels[side].slip_mps = 0.0f;
        }
        summary.wheels[side].couple_world_nm = rec.couple_world_nm;
        summary.wheels[side].rolling_dissipation_j = rec.rolling_dissipation_j;
        summary.wheels[side].radial_energy_j = sum_elastic_energy[side];
        summary.wheels[side].shear_energy_j = rec.shear_energy;
        summary.wheels[side].maxwell_energy_j = 0.0f;
        summary.wheels[side].rim_energy_j = 0.0f;
        summary.wheels[side].radial_loss_j = rec.radial_power * dt_s;
        summary.wheels[side].brush_and_rolling_loss_j =
            rec.losses + rec.rolling_dissipation_j;
        summary.wheels[side].rolling_moment_nm = rec.couple_world_nm;
        return Status::ok;
    }
};

struct GpuCompliantTerrainFn {
    PolylineView<float> pl{};

    BIKE_HD Status operator()(
        float cx,
        float cz,
        float radius,
        bool has_prev_seg,
        i32 prev_seg,
        BIKE_REF(PolylineContactResult<float>) out,
        BIKE_REF(ErrCode) err
    ) noexcept {
        return scan_polyline_contact<float, GpuToolkit, NoMultiSupport>(
            pl, cx, cz, radius, has_prev_seg, prev_seg, out, err);
    }
};

template <class XposPtr, class XanchorPtr, class XaxisPtr>
struct GpuCompliantJacFn {
    GpuWheelChain chains[2]{};
    i32 nv = 0;
    XposPtr xpos_w = nullptr;
    XanchorPtr xanchor_w = nullptr;
    XaxisPtr xaxis_w = nullptr;
    float jp_point[3 * kMaxNv]{};

    template <class QvelPtr>
    BIKE_HD Status operator()(
        i32 side,
        BIKE_CREF(CompliantJacInput<float>) in,
        QvelPtr qvel,
        BIKE_REF(CompliantJacOutput<float>) out,
        BIKE_REF(ErrCode) err
    ) noexcept {
        const GpuWheelChain local_chain = chains[side];
        const float pt[3] = {in.point.x, in.point.y, in.point.z};
        const float ct[3] = {in.center.x, in.center.y, in.center.z};
        float jp_center[3 * kMaxNv]{};
        BIKE_THREAD float *null_jr = nullptr;
        Status st = chain_point_jacobian<float, GpuToolkit, false>(
            local_chain, xpos_w, xanchor_w, xaxis_w, local_chain.wheel_body_id,
            nv, local_chain.njnt, pt, jp_point, null_jr, err);
        if (st != Status::ok) {
            return st;
        }
        st = chain_point_jacobian<float, GpuToolkit, false>(
            local_chain, xpos_w, xanchor_w, xaxis_w, local_chain.wheel_body_id,
            nv, local_chain.njnt, ct, jp_center, null_jr, err);
        if (st != Status::ok) {
            return st;
        }
        float vp[3]{};
        float vc[3]{};
        GpuToolkit::gemv_n3(nv, jp_point, qvel, vp);
        GpuToolkit::gemv_n3(nv, jp_center, qvel, vc);
        out.velocity = TyreVec3<float>{.x = vp[0], .y = vp[1], .z = vp[2]};
        out.center_velocity = TyreVec3<float>{.x = vc[0], .y = vc[1], .z = vc[2]};
        err = ErrCode::none;
        return Status::ok;
    }
};

struct GpuCompliantQfrcAccumFn {
    BIKE_THREAD const float *jp_point = nullptr;
    i32 nv = 0;

    template <class OutQfrcPtr>
    BIKE_HD Status operator()(
        i32,
        BIKE_CREF(TyreVec3<float>),
        BIKE_CREF(TyreVec3<float>) force_world,
        OutQfrcPtr out_qfrc,
        BIKE_REF(ErrCode) err
    ) noexcept {
        const float f[3] = {force_world.x, force_world.y, force_world.z};
        float tmp[kMaxNv]{};
        GpuToolkit::gemv_t3(nv, jp_point, f, tmp);
        for (i32 i = 0; i < nv; ++i) {
            out_qfrc[i] += tmp[i];
        }
        err = ErrCode::none;
        return Status::ok;
    }
};

struct GpuDistProfileHeightFn {
    SignedProfileView<float> sp{};

    BIKE_HD Status operator()(
        float x,
        BIKE_REF(float) out_h,
        BIKE_REF(ErrCode) err
    ) noexcept {
        return profile_height<float, GpuToolkit>(sp, x, out_h, err);
    }
};

struct GpuDistSignedQueryFn {
    SignedProfileView<float> sp{};

    BIKE_HD Status operator()(
        BIKE_CREF(TyreXZ<float>) query_xz,
        BIKE_REF(PointDistanceResult<float>) out,
        BIKE_REF(ErrCode) err
    ) noexcept {
        return signed_profile_query<float, GpuToolkit>(sp, query_xz.x, query_xz.z, out, err);
    }
};

template <class XposPtr, class XanchorPtr, class XaxisPtr>
struct GpuDistCenterJacFn {
    GpuWheelChain chain{};
    i32 nv = 0;
    XposPtr xpos_w = nullptr;
    XanchorPtr xanchor_w = nullptr;
    XaxisPtr xaxis_w = nullptr;

    BIKE_HD Status operator()(
        BIKE_CREF(TyreVec3<float>) center_pt,
        BIKE_THREAD float *out_jc,
        BIKE_THREAD float *out_jr,
        BIKE_REF(ErrCode) err
    ) noexcept {
        const GpuWheelChain local_chain = chain;
        const float ct[3] = {center_pt.x, center_pt.y, center_pt.z};
        return chain_point_jacobian<float, GpuToolkit, true>(
            local_chain, xpos_w, xanchor_w, xaxis_w, local_chain.wheel_body_id,
            nv, local_chain.njnt, ct, out_jc, out_jr, err);
    }
};

#if defined(__METAL_VERSION__)
#define BIKE_TYRE_NOINLINE_HD inline
#elif defined(__CUDACC__)
#define BIKE_TYRE_NOINLINE_HD __host__ __device__ __noinline__
#elif defined(__clang__) || defined(__GNUC__)
#define BIKE_TYRE_NOINLINE_HD __attribute__((noinline)) inline
#else
#define BIKE_TYRE_NOINLINE_HD inline
#endif

template <
    class XposPtr,
    class XanchorPtr,
    class XaxisPtr,
    class QvelPtr,
    class QfrcPtr,
    class CompliantStatePtr>
BIKE_TYRE_NOINLINE_HD Status gpu_step_compliant_both_wheels(
    BIKE_DEVICE const GpuTyreModel *model,
    BIKE_CREF(GpuTerrainTableDesc) terrain_desc,
    BIKE_DEVICE const float *terrain_f32,
    BIKE_DEVICE const i32 *terrain_i32,
    float tire_stiff_scale,
    float dt_s,
    XposPtr xpos_w,
    XanchorPtr xanchor_w,
    XaxisPtr xaxis_w,
    QvelPtr qvel_w,
    CompliantStatePtr compliant_states_w,
    QfrcPtr out_qfrc_w,
    BIKE_REF(GpuTyreStepOutput) out_summary,
    BIKE_REF(ErrCode) err
) noexcept {
    if (compliant_states_w == nullptr) {
        err = ErrCode::invalid_prev_seg;
        return Status::invalid_argument;
    }

    const ScaledTyreModel scaled = make_scaled_tyre_model(model, tire_stiff_scale);
    GpuWheelSummaryIo io{};
    io.model = model;
    io.tire_stiff_scale = tire_stiff_scale;
    io.dt_s = dt_s;
    io.tk_scaled[0] = scaled.wheels[0].tangent_k_n_m;
    io.tk_scaled[1] = scaled.wheels[1].tangent_k_n_m;
    io.nominal_radius[0] = scaled.wheels[0].radius_m;
    io.nominal_radius[1] = scaled.wheels[1].radius_m;

    GpuCompliantTerrainFn terrain_fn{};
    terrain_fn.pl = make_polyline_view(terrain_desc, terrain_f32, terrain_i32);

    GpuCompliantJacFn<XposPtr, XanchorPtr, XaxisPtr> jac_fn{};
    jac_fn.chains[0] = model->wheels[0].chain;
    jac_fn.chains[1] = model->wheels[1].chain;
    jac_fn.nv = scaled.nv;
    jac_fn.xpos_w = xpos_w;
    jac_fn.xanchor_w = xanchor_w;
    jac_fn.xaxis_w = xaxis_w;

    GpuCompliantQfrcAccumFn accum_fn{jac_fn.jp_point, scaled.nv};

    CompliantWheelStatePod<float> next_st[2]{};
    float qfrc_delta[kMaxNv]{};

    for (i32 side = 0; side < 2; ++side) {
        const i32 bid = jac_fn.chains[side].wheel_body_id;
        const float center[3] = {
            xpos_w[3 * bid + 0],
            xpos_w[3 * bid + 1],
            xpos_w[3 * bid + 2]
        };
        io.wheel_center[side] = TyreVec3<float>{.x = center[0], .y = center[1], .z = center[2]};
        CompliantWheelStatePod<float> prev_st = compliant_states_w[side];
        const Status st = step_compliant_wheel<float, GpuToolkit>(
            &scaled,
            side,
            dt_s,
            center,
            qvel_w,
            &prev_st,
            &next_st[side],
            qfrc_delta,
            terrain_fn,
            jac_fn,
            accum_fn,
            io,
            err);
        if (st != Status::ok) {
            return st;
        }
    }

    compliant_states_w[0] = next_st[0];
    compliant_states_w[1] = next_st[1];
    for (i32 i = 0; i < scaled.nv; ++i) {
        out_qfrc_w[i] += qfrc_delta[i];
    }
    out_summary = io.summary;
    return Status::ok;
}

using GpuDistWorkspace = GpuDistributedWorkspace<float, kMaxGpuTyreStations, kMaxNv>;

template <
    class XposPtr,
    class XmatPtr,
    class XanchorPtr,
    class XaxisPtr,
    class QvelPtr,
    class DistStatePtr>
BIKE_TYRE_NOINLINE_HD Status gpu_invoke_step_distributed_wheel(
    const BIKE_THREAD ScaledTyreModel *scaled,
    i32 side,
    float dt_s,
    const BIKE_THREAD float center[3],
    XmatPtr xmat_w,
    QvelPtr qvel_w,
    BIKE_DEVICE const float *local_xyz,
    DistStatePtr prev_states_w,
    BIKE_THREAD StationStatePod<float> *next_states_stage,
    BIKE_THREAD float *qfrc_delta,
    BIKE_REF(GpuDistWorkspace) ws,
    const BIKE_THREAD SignedProfileView<float> &sp,
    const BIKE_THREAD GpuWheelChain &side_chain,
    XposPtr xpos_w,
    XanchorPtr xanchor_w,
    XaxisPtr xaxis_w,
    BIKE_REF(GpuWheelSummaryIo) io,
    BIKE_REF(ErrCode) err
) noexcept {
    BIKE_DEVICE const float *no_weights = nullptr;
    GpuDistProfileHeightFn height_fn{sp};
    GpuDistSignedQueryFn query_fn{sp};
    GpuDistCenterJacFn<XposPtr, XanchorPtr, XaxisPtr> cjac_fn{
        side_chain, scaled->nv, xpos_w, xanchor_w, xaxis_w};
    return step_distributed_wheel<float, GpuToolkit>(
        scaled,
        side,
        dt_s,
        center,
        xmat_w,
        qvel_w,
        local_xyz,
        no_weights,
        prev_states_w,
        next_states_stage,
        qfrc_delta,
        ws,
        height_fn,
        query_fn,
        cjac_fn,
        io,
        err);
}

template <
    class XposPtr,
    class XmatPtr,
    class XanchorPtr,
    class XaxisPtr,
    class QvelPtr,
    class DistStatePtr>
BIKE_TYRE_NOINLINE_HD Status gpu_step_distributed_one_wheel(
    const BIKE_THREAD ScaledTyreModel *scaled,
    i32 side,
    const BIKE_THREAD GpuWheelChain &side_chain,
    BIKE_DEVICE const float *local_xyz,
    const BIKE_THREAD SignedProfileView<float> &sp,
    float dt_s,
    XposPtr xpos_w,
    XmatPtr xmat_w,
    XanchorPtr xanchor_w,
    XaxisPtr xaxis_w,
    QvelPtr qvel_w,
    DistStatePtr prev_states_w,
    BIKE_THREAD StationStatePod<float> *next_states_stage,
    BIKE_THREAD float *qfrc_delta,
    BIKE_REF(GpuWheelSummaryIo) io,
    BIKE_REF(ErrCode) err
) noexcept {
    GpuDistributedWorkspace<float, kMaxGpuTyreStations, kMaxNv> ws{};
    const i32 bid = side_chain.wheel_body_id;
    const float center[3] = {
        xpos_w[3 * bid + 0],
        xpos_w[3 * bid + 1],
        xpos_w[3 * bid + 2]
    };
    io.wheel_center[side] = TyreVec3<float>{.x = center[0], .y = center[1], .z = center[2]};
    return gpu_invoke_step_distributed_wheel(
        scaled,
        side,
        dt_s,
        center,
        xmat_w + 9 * bid,
        qvel_w,
        local_xyz,
        prev_states_w,
        next_states_stage,
        qfrc_delta,
        ws,
        sp,
        side_chain,
        xpos_w,
        xanchor_w,
        xaxis_w,
        io,
        err);
}

template <
    class XposPtr,
    class XmatPtr,
    class XanchorPtr,
    class XaxisPtr,
    class QvelPtr,
    class QfrcPtr,
    class DistStatePtr>
BIKE_TYRE_NOINLINE_HD Status gpu_step_distributed_side1(
    BIKE_DEVICE const GpuTyreModel *model,
    const BIKE_THREAD ScaledTyreModel *scaled,
    const BIKE_THREAD SignedProfileView<float> &sp,
    float dt_s,
    XposPtr xpos_w,
    XmatPtr xmat_w,
    XanchorPtr xanchor_w,
    XaxisPtr xaxis_w,
    QvelPtr qvel_w,
    DistStatePtr dist_states_w,
    const BIKE_THREAD float *qfrc_delta0,
    QfrcPtr out_qfrc_w,
    BIKE_REF(GpuWheelSummaryIo) io,
    BIKE_REF(ErrCode) err
) noexcept {
    StationStatePod<float> next1[kMaxGpuTyreStations]{};
    float qfrc_delta1[kMaxNv]{};
    const GpuWheelChain side_chain = model->wheels[1].chain;
    const Status st = gpu_step_distributed_one_wheel(
        scaled,
        1,
        side_chain,
        model->wheels[1].local_xyz,
        sp,
        dt_s,
        xpos_w,
        xmat_w,
        xanchor_w,
        xaxis_w,
        qvel_w,
        dist_states_w + static_cast<long>(kMaxGpuTyreStations),
        next1,
        qfrc_delta1,
        io,
        err);
    if (st != Status::ok) {
        return st;
    }

    const i32 scnt = scaled->station_count;
    for (i32 i = 0; i < scnt; ++i) {
        dist_states_w[static_cast<long>(kMaxGpuTyreStations) + i] = next1[i];
    }
    const i32 nv = scaled->nv;
    for (i32 i = 0; i < nv; ++i) {
        out_qfrc_w[i] = (out_qfrc_w[i] + qfrc_delta0[i]) + qfrc_delta1[i];
    }
    return Status::ok;
}

template <
    class XposPtr,
    class XmatPtr,
    class XanchorPtr,
    class XaxisPtr,
    class QvelPtr,
    class QfrcPtr,
    class DistStatePtr>
BIKE_TYRE_NOINLINE_HD Status gpu_step_distributed_both_wheels_impl(
    BIKE_DEVICE const GpuTyreModel *model,
    const BIKE_THREAD ScaledTyreModel *scaled,
    float tire_stiff_scale,
    BIKE_CREF(GpuTerrainTableDesc) terrain_desc,
    BIKE_DEVICE const float *terrain_f32,
    float dt_s,
    XposPtr xpos_w,
    XmatPtr xmat_w,
    XanchorPtr xanchor_w,
    XaxisPtr xaxis_w,
    QvelPtr qvel_w,
    DistStatePtr dist_states_w,
    QfrcPtr out_qfrc_w,
    BIKE_REF(GpuTyreStepOutput) out_summary,
    BIKE_REF(ErrCode) err
) noexcept {
    GpuWheelSummaryIo io{};
    io.model = model;
    io.tire_stiff_scale = tire_stiff_scale;
    io.dt_s = dt_s;
    io.tk_scaled[0] = scaled->wheels[0].tangent_k_n_m;
    io.tk_scaled[1] = scaled->wheels[1].tangent_k_n_m;
    io.nominal_radius[0] = scaled->wheels[0].radius_m;
    io.nominal_radius[1] = scaled->wheels[1].radius_m;

    const SignedProfileView<float> sp = make_signed_profile_view(terrain_desc, terrain_f32);
    StationStatePod<float> next0[kMaxGpuTyreStations]{};
    float qfrc_delta0[kMaxNv]{};
    const GpuWheelChain side0_chain = model->wheels[0].chain;

    Status st = gpu_step_distributed_one_wheel(
        scaled,
        0,
        side0_chain,
        model->wheels[0].local_xyz,
        sp,
        dt_s,
        xpos_w,
        xmat_w,
        xanchor_w,
        xaxis_w,
        qvel_w,
        dist_states_w,
        next0,
        qfrc_delta0,
        io,
        err);
    if (st != Status::ok) {
        return st;
    }

    st = gpu_step_distributed_side1(
        model,
        scaled,
        sp,
        dt_s,
        xpos_w,
        xmat_w,
        xanchor_w,
        xaxis_w,
        qvel_w,
        dist_states_w,
        qfrc_delta0,
        out_qfrc_w,
        io,
        err);
    if (st != Status::ok) {
        return st;
    }

    const i32 scnt = scaled->station_count;
    for (i32 i = 0; i < scnt; ++i) {
        dist_states_w[i] = next0[i];
    }
    out_summary = io.summary;
    return Status::ok;
}

template <
    class XposPtr,
    class XmatPtr,
    class XanchorPtr,
    class XaxisPtr,
    class QvelPtr,
    class QfrcPtr,
    class DistStatePtr>
BIKE_TYRE_NOINLINE_HD Status gpu_step_distributed_both_wheels(
    BIKE_DEVICE const GpuTyreModel *model,
    float tire_stiff_scale,
    BIKE_CREF(GpuTerrainTableDesc) terrain_desc,
    BIKE_DEVICE const float *terrain_f32,
    float dt_s,
    XposPtr xpos_w,
    XmatPtr xmat_w,
    XanchorPtr xanchor_w,
    XaxisPtr xaxis_w,
    QvelPtr qvel_w,
    DistStatePtr dist_states_w,
    QfrcPtr out_qfrc_w,
    BIKE_REF(GpuTyreStepOutput) out_summary,
    BIKE_REF(ErrCode) err
) noexcept {
    if (xmat_w == nullptr || dist_states_w == nullptr) {
        err = ErrCode::out_of_bounds_chain;
        return Status::invalid_argument;
    }

    const ScaledTyreModel scaled = make_scaled_tyre_model(model, tire_stiff_scale);
    return gpu_step_distributed_both_wheels_impl(
        model,
        &scaled,
        tire_stiff_scale,
        terrain_desc,
        terrain_f32,
        dt_s,
        xpos_w,
        xmat_w,
        xanchor_w,
        xaxis_w,
        qvel_w,
        dist_states_w,
        out_qfrc_w,
        out_summary,
        err);
}

template <
    class XposPtr,
    class XmatPtr,
    class XanchorPtr,
    class XaxisPtr,
    class QvelPtr,
    class QfrcPtr,
    class PneumaticStatePtr,
    class PneumaticScratchPtr>
BIKE_TYRE_NOINLINE_HD Status gpu_step_pneumatic_both_wheels(
    BIKE_DEVICE const GpuTyreModel *model,
    BIKE_CREF(GpuTerrainTableDesc) terrain_desc,
    BIKE_DEVICE const float *terrain_f32,
    float dt_s,
    XposPtr xpos_w,
    XmatPtr xmat_w,
    XanchorPtr xanchor_w,
    XaxisPtr xaxis_w,
    QvelPtr qvel_w,
    PneumaticStatePtr pneum_states_w,
    PneumaticScratchPtr pneum_scratch_w,
    QfrcPtr out_qfrc_w,
    BIKE_REF(GpuTyreStepOutput) out_summary,
    BIKE_REF(ErrCode) err
) noexcept {
    if (xmat_w == nullptr || pneum_states_w == nullptr || pneum_scratch_w == nullptr) {
        err = ErrCode::out_of_bounds_chain;
        return Status::invalid_argument;
    }

    PneumaticSurfaceMap<float> pmap{};
    pmap.default_surface.mu_peak = model->surface_map.default_surface.mu_peak;
    pmap.default_surface.mu_slide = model->surface_map.default_surface.mu_slide;
    pmap.default_surface.slip_stiffness_per_load =
        model->surface_map.default_surface.slip_stiffness_per_load;
    pmap.default_surface.stribeck_speed_mps =
        model->surface_map.default_surface.stribeck_speed_mps;
    pmap.default_surface.crr_delta = model->surface_map.default_surface.crr_delta;
    pmap.default_surface.load_sensitivity =
        model->surface_map.default_surface.load_sensitivity;
    pmap.default_surface_id = model->surface_map.default_surface.surface_id;
    pmap.has_surface_map = model->surface_map.has_surface_map;
    pmap.section_count = model->surface_map.section_count;
    const i32 sec_count =
        pmap.section_count < kMaxSurfaceSections ? pmap.section_count : kMaxSurfaceSections;
    for (i32 i = 0; i < sec_count; ++i) {
        pmap.sections[i].start_m = model->surface_map.sections[i].start_m;
        pmap.sections[i].end_m = model->surface_map.sections[i].end_m;
        pmap.sections[i].blend_m = model->surface_map.sections[i].blend_m;
        pmap.sections[i].surface_id =
            model->surface_map.sections[i].surface.surface_id;
        pmap.sections[i].surface.mu_peak =
            model->surface_map.sections[i].surface.mu_peak;
        pmap.sections[i].surface.mu_slide =
            model->surface_map.sections[i].surface.mu_slide;
        pmap.sections[i].surface.slip_stiffness_per_load =
            model->surface_map.sections[i].surface.slip_stiffness_per_load;
        pmap.sections[i].surface.stribeck_speed_mps =
            model->surface_map.sections[i].surface.stribeck_speed_mps;
        pmap.sections[i].surface.crr_delta =
            model->surface_map.sections[i].surface.crr_delta;
        pmap.sections[i].surface.load_sensitivity =
            model->surface_map.sections[i].surface.load_sensitivity;
    }

    BIKE_DEVICE const float *road_px = terrain_f32 + terrain_desc.px_offset;
    BIKE_DEVICE const float *road_pz = terrain_f32 + terrain_desc.pz_offset;
    const i32 road_num_vertices = terrain_desc.polyline_vertex_count;
    const bool uniform_grid = (model->road_is_uniform_grid != 0);

    float qfrc_delta[kMaxNv]{};
    GpuTyreStepOutput summary{};

    for (i32 side = 0; side < 2; ++side) {
        BIKE_DEVICE const PneumaticMaterialView<float> *pv =
            &model->wheels[side].pneumatic;
        GpuPneumaticSpecRef spec{};
        spec.tyre.outer_radius_mm = pv->outer_radius_mm;
        spec.tyre.rim_radius_mm = pv->rim_radius_mm;
        spec.tyre.section_height_mm = pv->section_height_mm;
        spec.tyre.pressure_bar = pv->pressure_bar;
        spec.tyre.crr_reference = pv->crr_reference;
        spec.tyre.casing_width_mm = pv->casing_width_mm;
        spec.tyre.compressed_casing_mm = pv->compressed_casing_mm;
        spec.tyre.area_factor = pv->area_factor;
        spec.tyre.carcass_stiffness_n_mm2 = pv->carcass_stiffness_n_mm2;
        spec.tyre.contact_length_factor = pv->contact_length_factor;
        spec.tyre.rate_stiffening = pv->rate_stiffening;
        spec.tyre.rate_relaxation_s = pv->rate_relaxation_s;
        spec.tyre.loss_factor = pv->loss_factor;
        spec.tyre.hysteresis_rate_eps_mps = pv->hysteresis_rate_eps_mps;
        spec.tyre.relaxation_length_mm = pv->relaxation_length_mm;
        spec.tyre.rim_stiffness_n_mm2 = pv->rim_stiffness_n_mm2;
        spec.tyre.tread_loss_crr = pv->tread_loss_crr;
        spec.ring.n_rays = pv->n_rays;
        spec.ring.half_angle_rad = pv->half_angle_rad;
        spec.ring.dtheta_rad = pv->dtheta_rad;
        spec.ring.theta_rad = pv->theta_rad;
        spec.ring.ux = pv->ux;
        spec.ring.uz = pv->uz;
        spec.chain = model->wheels[side].chain;
        spec.wheel_body_id = spec.chain.wheel_body_id;
        spec.nv = model->nv;
        spec.njnt = model->njnt;
        spec.discretised_brush = pv->discretised_brush;

        if (pneum_states_w[side].carcass.initialized == 0 &&
            pneum_states_w[side].carcass.n_rays == 0) {
            init_pneumatic_wheel_state<float>(spec.ring.n_rays, &pneum_states_w[side]);
        }

        GpuPneumaticWheelOutput<float> p_out{};
        const Status st = step_pneumatic_wheel<float, GpuToolkit>(
            &spec,
            &pmap,
            dt_s,
            road_px,
            road_pz,
            road_num_vertices,
            model->road_x0_m,
            model->road_dx_m,
            uniform_grid,
            xpos_w,
            xmat_w,
            xanchor_w,
            xaxis_w,
            qvel_w,
            &pneum_states_w[side],
            &pneum_states_w[side],
            qfrc_delta,
            p_out,
            pneum_scratch_w,
            err);
        if (st != Status::ok) {
            return st;
        }

        summary.active_station_counts[side] = p_out.patch_count;
        summary.wheels[side].normal_load_n = p_out.normal_load_n;
        summary.wheels[side].effective_radius_m = p_out.effective_radius_m;
        summary.wheels[side].slip_mps = p_out.slip_mps;
        summary.wheels[side].couple_world_nm = p_out.rolling_moment_nm;
        summary.wheels[side].rolling_dissipation_j = p_out.brush_and_rolling_loss_j;
        summary.wheels[side].radial_energy_j = p_out.radial_energy_j;
        summary.wheels[side].shear_energy_j = p_out.shear_energy_j;
        summary.wheels[side].maxwell_energy_j = p_out.maxwell_energy_j;
        summary.wheels[side].rim_energy_j = p_out.rim_energy_j;
        summary.wheels[side].radial_loss_j = p_out.carcass_loss_j;
        summary.wheels[side].brush_and_rolling_loss_j = p_out.brush_and_rolling_loss_j;
        summary.wheels[side].rolling_moment_nm = p_out.rolling_moment_nm;
    }

    for (i32 i = 0; i < model->nv; ++i) {
        out_qfrc_w[i] += qfrc_delta[i];
    }
    out_summary = summary;
    return Status::ok;
}

template <
    class XposPtr,
    class XmatPtr,
    class XanchorPtr,
    class XaxisPtr,
    class QvelPtr,
    class QfrcPtr,
    class CompliantStatePtr,
    class DistStatePtr,
    class PneumaticStatePtr = BIKE_DEVICE GpuPneumaticWheelState<float> *,
    class PneumaticScratchPtr = BIKE_DEVICE GpuPneumaticStepScratch<float> *>
BIKE_HD Status gpu_step_both_wheels_tyre(
    BIKE_DEVICE const GpuTyreModel *model,
    BIKE_CREF(GpuTerrainTableDesc) terrain_desc,
    BIKE_DEVICE const float *terrain_f32,
    BIKE_DEVICE const i32 *terrain_i32,
    float tire_stiff_scale,
    float dt_s,
    XposPtr xpos_w,
    XmatPtr xmat_w,
    XanchorPtr xanchor_w,
    XaxisPtr xaxis_w,
    QvelPtr qvel_w,
    CompliantStatePtr compliant_states_w,
    DistStatePtr dist_states_w,
    QfrcPtr out_qfrc_w,
    BIKE_REF(GpuTyreStepOutput) out_summary,
    BIKE_REF(ErrCode) err,
    bool check_monotonic = false,
    PneumaticStatePtr pneum_states_w =
        static_cast<BIKE_DEVICE GpuPneumaticWheelState<float> *>(nullptr),
    PneumaticScratchPtr pneum_scratch_w =
        static_cast<BIKE_DEVICE GpuPneumaticStepScratch<float> *>(nullptr)
) noexcept {
    err = ErrCode::none;
    out_summary = GpuTyreStepOutput{};

    Status st = validate_gpu_tyre_model_device(model, 0, 0, 0, err);
    if (st != Status::ok) {
        return st;
    }
    if (xpos_w == nullptr || xanchor_w == nullptr || xaxis_w == nullptr ||
        qvel_w == nullptr || out_qfrc_w == nullptr ||
        !GpuToolkit::isfinite(dt_s) || dt_s <= 0.0f) {
        err = ErrCode::out_of_bounds_chain;
        return Status::invalid_argument;
    }

    const bool need_polyline =
        (model->backend == TyreBackendKind::compliant_2d ||
         model->backend == TyreBackendKind::pneumatic);
    const bool need_signed = (model->backend == TyreBackendKind::distributed_2d_reference);
    st = validate_terrain_table_desc(
        terrain_desc, terrain_f32, terrain_i32,
        need_polyline, need_signed, check_monotonic, err);
    if (st != Status::ok) {
        return st;
    }

    if (model->backend == TyreBackendKind::compliant_2d) {
        st = gpu_step_compliant_both_wheels(
            model,
            terrain_desc,
            terrain_f32,
            terrain_i32,
            tire_stiff_scale,
            dt_s,
            xpos_w,
            xanchor_w,
            xaxis_w,
            qvel_w,
            compliant_states_w,
            out_qfrc_w,
            out_summary,
            err);
    } else if (model->backend == TyreBackendKind::distributed_2d_reference) {
        st = gpu_step_distributed_both_wheels(
            model,
            tire_stiff_scale,
            terrain_desc,
            terrain_f32,
            dt_s,
            xpos_w,
            xmat_w,
            xanchor_w,
            xaxis_w,
            qvel_w,
            dist_states_w,
            out_qfrc_w,
            out_summary,
            err);
    } else {
        st = gpu_step_pneumatic_both_wheels(
            model,
            terrain_desc,
            terrain_f32,
            dt_s,
            xpos_w,
            xmat_w,
            xanchor_w,
            xaxis_w,
            qvel_w,
            pneum_states_w,
            pneum_scratch_w,
            out_qfrc_w,
            out_summary,
            err);
    }
    if (st == Status::ok) {
        out_summary.radial_energy_j =
            out_summary.wheels[0].radial_energy_j + out_summary.wheels[1].radial_energy_j;
        out_summary.shear_energy_j =
            out_summary.wheels[0].shear_energy_j + out_summary.wheels[1].shear_energy_j;
        out_summary.maxwell_energy_j =
            out_summary.wheels[0].maxwell_energy_j + out_summary.wheels[1].maxwell_energy_j;
        out_summary.rim_energy_j =
            out_summary.wheels[0].rim_energy_j + out_summary.wheels[1].rim_energy_j;
        out_summary.radial_loss_j =
            out_summary.wheels[0].radial_loss_j + out_summary.wheels[1].radial_loss_j;
        out_summary.brush_and_rolling_loss_j =
            out_summary.wheels[0].brush_and_rolling_loss_j +
            out_summary.wheels[1].brush_and_rolling_loss_j;
        out_summary.rolling_moment_nm =
            out_summary.wheels[0].rolling_moment_nm +
            out_summary.wheels[1].rolling_moment_nm;
    }
    return st;
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
