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
#include "tyre_types.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: templated pointer parameters accept device/constant/thread buffers in MSL without address-space cast errors. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

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
};

struct GpuTyreStepOutput {
    GpuWheelSummary wheels[2]{};
    i32 active_station_counts[2]{};
};

struct ScaledTyreWheelSpec {
    float radius_m = 0.0f;
    float tangent_k_n_m = 0.0f;
    float tire_mu = 0.0f;
    float relaxation_length_m = 0.0f;
    CompliantMaterialView<float> compliant{};
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
        model->backend != TyreBackendKind::distributed_2d_reference) {
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
            if (model->wheels[side].compliant.has_tabulated_radial != 0 ||
                !GpuToolkit::isfinite(model->wheels[side].compliant.radial_k_n_m) ||
                model->wheels[side].compliant.radial_k_n_m < 0.0f) {
                err = ErrCode::missing_material;
                return Status::invalid_argument;
            }
        } else {
            const i32 kcnt = model->wheels[side].distributed_density.knot_count;
            if (kcnt < 2 || kcnt > static_cast<i32>(kMaxHingeKnots)) {
                err = ErrCode::missing_material;
                return Status::out_of_bounds;
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
    pl.height_changes = terrain_i32 + desc.height_changes_offset;
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
        out.wheels[side].compliant = model->wheels[side].compliant;
        out.wheels[side].compliant.radial_k_n_m *= scale;
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
    float nominal_radius[2]{};
    TyreVec3<float> wheel_center[2]{};
    float sum_load[2]{};
    float sum_weighted_drop[2]{};
    float sum_unweighted_drop[2]{};
    GpuTyreStepOutput summary{};

    BIKE_HD bool eval_radial(
        i32,
        BIKE_CREF(RadialEvalInput<float>),
        BIKE_REF(RadialEvalOutput<float>),
        BIKE_REF(Status),
        BIKE_REF(ErrCode)
    ) noexcept {
        return false;
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
            summary.active_station_counts[side] = 1;
        } else {
            summary.wheels[side].effective_radius_m = 0.0f;
            summary.active_station_counts[side] = 0;
        }
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
        } else if (sum_load[side] > 0.0f) {
            summary.wheels[side].effective_radius_m =
                GpuToolkit::max(0.0f, sum_weighted_drop[side] / sum_load[side]);
        } else {
            summary.wheels[side].effective_radius_m = GpuToolkit::max(
                0.0f, sum_unweighted_drop[side] / static_cast<float>(rec.active_count));
        }
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

template <
    class XposPtr,
    class XmatPtr,
    class XanchorPtr,
    class XaxisPtr,
    class QvelPtr,
    class QfrcPtr,
    class CompliantStatePtr,
    class DistStatePtr>
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
    bool check_monotonic = false
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

    const bool need_polyline = (model->backend == TyreBackendKind::compliant_2d);
    const bool need_signed = (model->backend == TyreBackendKind::distributed_2d_reference);
    st = validate_terrain_table_desc(
        terrain_desc, terrain_f32, terrain_i32,
        need_polyline, need_signed, check_monotonic, err);
    if (st != Status::ok) {
        return st;
    }

    const ScaledTyreModel scaled = make_scaled_tyre_model(model, tire_stiff_scale);
    GpuWheelSummaryIo io{};
    io.nominal_radius[0] = scaled.wheels[0].radius_m;
    io.nominal_radius[1] = scaled.wheels[1].radius_m;

    if (model->backend == TyreBackendKind::compliant_2d) {
        if (compliant_states_w == nullptr) {
            err = ErrCode::invalid_prev_seg;
            return Status::invalid_argument;
        }
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

        for (i32 side = 0; side < 2; ++side) {
            const i32 bid = jac_fn.chains[side].wheel_body_id;
            const float center[3] = {
                xpos_w[3 * bid + 0],
                xpos_w[3 * bid + 1],
                xpos_w[3 * bid + 2]
            };
            io.wheel_center[side] = TyreVec3<float>{.x = center[0], .y = center[1], .z = center[2]};
            CompliantWheelStatePod<float> prev_st = compliant_states_w[side];
            CompliantWheelStatePod<float> next_st{};
            st = step_compliant_wheel<float, GpuToolkit>(
                &scaled,
                side,
                dt_s,
                center,
                qvel_w,
                &prev_st,
                &next_st,
                out_qfrc_w,
                terrain_fn,
                jac_fn,
                accum_fn,
                io,
                err);
            if (st != Status::ok) {
                return st;
            }
            compliant_states_w[side] = next_st;
        }
        out_summary = io.summary;
        return Status::ok;
    }

    if (xmat_w == nullptr || dist_states_w == nullptr) {
        err = ErrCode::out_of_bounds_chain;
        return Status::invalid_argument;
    }

    const SignedProfileView<float> sp = make_signed_profile_view(terrain_desc, terrain_f32);
    GpuDistProfileHeightFn height_fn{sp};
    GpuDistSignedQueryFn query_fn{sp};
    GpuDistributedWorkspace<float, kMaxGpuTyreStations, kMaxNv> ws{};
    BIKE_DEVICE const float *no_weights = nullptr;

    for (i32 side = 0; side < 2; ++side) {
        const GpuWheelChain side_chain = model->wheels[side].chain;
        const i32 bid = side_chain.wheel_body_id;
        const float center[3] = {
            xpos_w[3 * bid + 0],
            xpos_w[3 * bid + 1],
            xpos_w[3 * bid + 2]
        };
        io.wheel_center[side] = TyreVec3<float>{.x = center[0], .y = center[1], .z = center[2]};
        GpuDistCenterJacFn<XposPtr, XanchorPtr, XaxisPtr> cjac_fn{
            side_chain, scaled.nv, xpos_w, xanchor_w, xaxis_w};
        st = step_distributed_wheel<float, GpuToolkit>(
            &scaled,
            side,
            dt_s,
            center,
            xmat_w + 9 * bid,
            qvel_w,
            model->wheels[side].local_xyz,
            no_weights,
            dist_states_w + static_cast<long>(side) * static_cast<long>(kMaxGpuTyreStations),
            dist_states_w + static_cast<long>(side) * static_cast<long>(kMaxGpuTyreStations),
            out_qfrc_w,
            ws,
            height_fn,
            query_fn,
            cjac_fn,
            io,
            err);
        if (st != Status::ok) {
            return st;
        }
    }

    out_summary = io.summary;
    return Status::ok;
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
