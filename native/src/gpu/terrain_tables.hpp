// native/src/gpu/terrain_tables.hpp — Host-side packing and boundary validation
// for flat GPU terrain tables (GpuTerrainTableDesc + contiguous f32/i32 buffers).
#pragma once

#include "../physics_core/tyre_gpu_step.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <span>
#include <stdexcept>
#include <vector>

namespace bike::gpu {

struct HostPackedTerrain {
    bike::physics_core::GpuTerrainTableDesc desc{};
    std::vector<float> f32;
    std::vector<bike::physics_core::i32> i32;
};

struct HostPolylineSpans {
    std::span<const float> px;
    std::span<const float> pz;
    std::span<const float> seg_x;
    std::span<const float> seg_z;
    std::span<const float> seg_len_sq;
    std::span<const bike::physics_core::i32> height_changes;
    std::span<const float> left_x;
    std::span<const float> left_z;
    std::span<const float> right_x;
    std::span<const float> right_z;
    float maximum_z = 0.0f;
    float significant_delta_m = 0.0f;
    float significance_fraction = 0.0f;
    float normal_cosine = 0.0f;
};

struct HostSignedProfileSpans {
    std::span<const float> x;
    std::span<const float> z;
    std::span<const float> edge_x;
    std::span<const float> edge_z;
    std::span<const float> length2;
    std::span<const float> normal_x;
    std::span<const float> normal_z;
};

namespace detail {

inline bike::physics_core::i32 append_f32_span(
    std::vector<float> &dst,
    std::span<const float> src
) {
    const auto offset = static_cast<bike::physics_core::i32>(dst.size());
    dst.insert(dst.end(), src.begin(), src.end());
    return offset;
}

inline bike::physics_core::i32 append_i32_span(
    std::vector<bike::physics_core::i32> &dst,
    std::span<const bike::physics_core::i32> src
) {
    const auto offset = static_cast<bike::physics_core::i32>(dst.size());
    dst.insert(dst.end(), src.begin(), src.end());
    return offset;
}

} // namespace detail

inline HostPackedTerrain pack_terrain_tables(
    const HostPolylineSpans &pl,
    const HostSignedProfileSpans &sp
) {
    HostPackedTerrain out{};
    if (!pl.px.empty()) {
        if (pl.px.size() < 2U) {
            throw std::invalid_argument("polyline terrain requires at least 2 vertices");
        }
        const std::size_t n = pl.px.size();
        const std::size_t s = n - 1U;
        if (pl.pz.size() != n || pl.seg_x.size() != s || pl.seg_z.size() != s ||
            pl.seg_len_sq.size() != s || pl.height_changes.size() != n ||
            pl.left_x.size() != n || pl.left_z.size() != n ||
            pl.right_x.size() != n || pl.right_z.size() != n) {
            throw std::invalid_argument("polyline span size mismatch");
        }
        out.desc.polyline_vertex_count = static_cast<bike::physics_core::i32>(n);
        out.desc.px_offset = detail::append_f32_span(out.f32, pl.px);
        out.desc.pz_offset = detail::append_f32_span(out.f32, pl.pz);
        out.desc.seg_x_offset = detail::append_f32_span(out.f32, pl.seg_x);
        out.desc.seg_z_offset = detail::append_f32_span(out.f32, pl.seg_z);
        out.desc.seg_len_sq_offset = detail::append_f32_span(out.f32, pl.seg_len_sq);
        out.desc.left_x_offset = detail::append_f32_span(out.f32, pl.left_x);
        out.desc.left_z_offset = detail::append_f32_span(out.f32, pl.left_z);
        out.desc.right_x_offset = detail::append_f32_span(out.f32, pl.right_x);
        out.desc.right_z_offset = detail::append_f32_span(out.f32, pl.right_z);
        out.desc.height_changes_offset = detail::append_i32_span(out.i32, pl.height_changes);
        out.desc.maximum_z = pl.maximum_z;
        out.desc.significant_delta_m = pl.significant_delta_m;
        out.desc.significance_fraction = pl.significance_fraction;
        out.desc.normal_cosine = pl.normal_cosine;
    }

    if (!sp.x.empty()) {
        if (sp.x.size() < 2U) {
            throw std::invalid_argument("signed-profile terrain requires at least 2 vertices");
        }
        const std::size_t n = sp.x.size();
        const std::size_t s = n - 1U;
        if (sp.z.size() != n || sp.edge_x.size() != s || sp.edge_z.size() != s ||
            sp.length2.size() != s || sp.normal_x.size() != s || sp.normal_z.size() != s) {
            throw std::invalid_argument("signed-profile span size mismatch");
        }
        out.desc.signed_vertex_count = static_cast<bike::physics_core::i32>(n);
        out.desc.sp_x_offset = detail::append_f32_span(out.f32, sp.x);
        out.desc.sp_z_offset = detail::append_f32_span(out.f32, sp.z);
        out.desc.sp_edge_x_offset = detail::append_f32_span(out.f32, sp.edge_x);
        out.desc.sp_edge_z_offset = detail::append_f32_span(out.f32, sp.edge_z);
        out.desc.sp_length2_offset = detail::append_f32_span(out.f32, sp.length2);
        out.desc.sp_normal_x_offset = detail::append_f32_span(out.f32, sp.normal_x);
        out.desc.sp_normal_z_offset = detail::append_f32_span(out.f32, sp.normal_z);
    }

    out.desc.f32_count = static_cast<bike::physics_core::i32>(out.f32.size());
    out.desc.i32_count = static_cast<bike::physics_core::i32>(out.i32.size());

    bike::physics_core::ErrCode err = bike::physics_core::ErrCode::none;
    if (bike::physics_core::validate_terrain_table_desc(
            out.desc,
            out.f32.empty() ? nullptr : out.f32.data(),
            out.i32.empty() ? nullptr : out.i32.data(),
            !pl.px.empty(),
            !sp.x.empty(),
            true,
            err) != bike::physics_core::Status::ok) {
        throw std::invalid_argument("packed terrain tables failed validation");
    }
    return out;
}

struct UniformTerrainXRange {
    float x_min = 0.0f;
    float x_max = 0.0f;
};

struct TerrainScanThresholds {
    float significant_delta_m = 0.005f;
    float significance_fraction = 0.05f;
    float distinct_normal_deg = 2.0f;
};

inline HostPackedTerrain pack_uniform_heightfield_to_terrain_tables(
    std::span<const float> z_src,
    UniformTerrainXRange x_range,
    TerrainScanThresholds thresholds = {}
) {
    const float x_min = x_range.x_min;
    const float x_max = x_range.x_max;
    const float significant_delta_m = thresholds.significant_delta_m;
    const float significance_fraction = thresholds.significance_fraction;
    const float distinct_normal_deg = thresholds.distinct_normal_deg;
    if (z_src.size() < 2U || !std::isfinite(x_min) || !std::isfinite(x_max) ||
        !(x_max > x_min)) {
        throw std::invalid_argument("invalid uniform heightfield inputs for terrain table packing");
    }

    const std::size_t cs = z_src.size();
    const std::size_t nseg = cs - 1U;

    std::vector<double> px_d(cs);
    std::vector<double> pz_d(cs);
    const double xmin_d = static_cast<double>(x_min);
    const double xmax_d = static_cast<double>(x_max);
    const double dx_d = (xmax_d - xmin_d) / static_cast<double>(nseg);

    std::vector<float> px(cs);
    std::vector<float> pz(cs);
    float max_z = z_src[0];
    for (std::size_t i = 0; i < cs; ++i) {
        if (!std::isfinite(z_src[i])) {
            throw std::invalid_argument("non-finite height sample in uniform heightfield");
        }
        px_d[i] = (i + 1U == cs) ? xmax_d : (xmin_d + static_cast<double>(i) * dx_d);
        pz_d[i] = static_cast<double>(z_src[i]);
        px[i] = static_cast<float>(px_d[i]);
        pz[i] = z_src[i];
        max_z = std::max(max_z, pz[i]);
    }
    for (std::size_t i = 0; i < nseg; ++i) {
        if (!(px[i + 1U] > px[i])) {
            throw std::invalid_argument("heightfield x grid is not strictly increasing in float32");
        }
    }

    std::vector<float> seg_x(nseg);
    std::vector<float> seg_z(nseg);
    std::vector<float> seg_len_sq(nseg);
    std::vector<float> norm_x(nseg);
    std::vector<float> norm_z(nseg);
    std::vector<bike::physics_core::i32> height_changes(cs);
    height_changes[0] = 0;

    for (std::size_t i = 0; i < nseg; ++i) {
        const double sx_d = px_d[i + 1U] - px_d[i];
        const double sz_d = pz_d[i + 1U] - pz_d[i];
        const double len2_d = sx_d * sx_d + sz_d * sz_d;
        const double len_d = std::sqrt(len2_d);
        seg_x[i] = static_cast<float>(sx_d);
        seg_z[i] = static_cast<float>(sz_d);
        seg_len_sq[i] = static_cast<float>(len2_d);
        norm_x[i] = static_cast<float>(-sz_d / len_d);
        norm_z[i] = static_cast<float>(sx_d / len_d);
        height_changes[i + 1U] = height_changes[i] + (seg_z[i] != 0.0f ? 1 : 0);
    }

    std::vector<float> left_x(cs, 0.0f);
    std::vector<float> left_z(cs, 0.0f);
    std::vector<float> right_x(cs, 0.0f);
    std::vector<float> right_z(cs, 0.0f);
    for (std::size_t v = 1U; v < cs; ++v) {
        left_x[v] = -seg_x[v - 1U];
        left_z[v] = -seg_z[v - 1U];
    }
    for (std::size_t v = 0; v < nseg; ++v) {
        right_x[v] = seg_x[v];
        right_z[v] = seg_z[v];
    }

    const HostPolylineSpans pl{
        .px = px,
        .pz = pz,
        .seg_x = seg_x,
        .seg_z = seg_z,
        .seg_len_sq = seg_len_sq,
        .height_changes = height_changes,
        .left_x = left_x,
        .left_z = left_z,
        .right_x = right_x,
        .right_z = right_z,
        .maximum_z = max_z,
        .significant_delta_m = significant_delta_m,
        .significance_fraction = significance_fraction,
        .normal_cosine = static_cast<float>(
            std::cos(static_cast<double>(distinct_normal_deg) * (std::numbers::pi / 180.0))),
    };
    const HostSignedProfileSpans sp{
        .x = px,
        .z = pz,
        .edge_x = seg_x,
        .edge_z = seg_z,
        .length2 = seg_len_sq,
        .normal_x = norm_x,
        .normal_z = norm_z,
    };
    return pack_terrain_tables(pl, sp);
}

inline int validate_host_tyre_and_terrain(
    const bike::physics_core::GpuTyreModel *model,
    int expected_nq,
    int expected_nv,
    int expected_nbody,
    const bike::physics_core::GpuTerrainTableDesc *terrain_desc,
    const float *terrain_f32,
    const bike::physics_core::i32 *terrain_i32
) noexcept {
    if (model == nullptr || terrain_desc == nullptr) {
        return -1;
    }
    bike::physics_core::ErrCode err = bike::physics_core::ErrCode::none;
    if (bike::physics_core::validate_gpu_tyre_model_device(
            model,
            expected_nq,
            expected_nv,
            expected_nbody,
            err) != bike::physics_core::Status::ok) {
        return -1;
    }
    const bool need_polyline =
        (model->backend == bike::physics_core::TyreBackendKind::compliant_2d);
    const bool need_signed =
        (model->backend == bike::physics_core::TyreBackendKind::distributed_2d_reference);
    if (bike::physics_core::validate_terrain_table_desc(
            *terrain_desc,
            terrain_f32,
            terrain_i32,
            need_polyline,
            need_signed,
            true,
            err) != bike::physics_core::Status::ok) {
        return -1;
    }
    return 0;
}

} // namespace bike::gpu
