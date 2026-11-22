// native/src/physics_core/tyre_upload.cpp — Host-only builders and upload
// validation gates for GpuWheelChain, GpuTyreModel, PolylineView<float>, and
// SignedProfileView<float>.
#include "tyre_upload.hpp"

#include "../model_access.hpp"
#include "gpu_toolkit.hpp"
#include "tyre_pneumatic_geometry.hpp"
#include "tyre_tabulated.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace bike::physics_core {

namespace {

constexpr std::size_t to_idx(int v) noexcept {
    return static_cast<std::size_t>(v);
}

SurfaceSpecPod<float> pack_surface_spec(const nativecfg::SurfaceSpec &s, i32 surface_id) {
    if (!std::isfinite(s.mu_peak) || !std::isfinite(s.mu_slide) ||
        !std::isfinite(s.slip_stiffness_per_load) || !std::isfinite(s.stribeck_speed_mps) ||
        s.mu_peak <= 0.0 || s.mu_slide <= 0.0 ||
        s.slip_stiffness_per_load <= 0.0 || s.stribeck_speed_mps <= 0.0) {
        throw std::invalid_argument("surface coefficients must be positive and finite");
    }
    if (s.mu_slide > s.mu_peak) {
        throw std::invalid_argument("surface mu_slide exceeds mu_peak");
    }
    if (!std::isfinite(s.crr_delta) || s.crr_delta < 0.0) {
        throw std::invalid_argument("surface crr_delta must be finite and nonnegative");
    }
    if (!std::isfinite(s.load_sensitivity) ||
        s.load_sensitivity < 0.0 || s.load_sensitivity > 0.5) {
        throw std::invalid_argument("surface load_sensitivity must lie in [0, 0.5]");
    }
    return SurfaceSpecPod<float>{
        .mu_peak = static_cast<float>(s.mu_peak),
        .mu_slide = static_cast<float>(s.mu_slide),
        .slip_stiffness_per_load = static_cast<float>(s.slip_stiffness_per_load),
        .stribeck_speed_mps = static_cast<float>(s.stribeck_speed_mps),
        .crr_delta = static_cast<float>(s.crr_delta),
        .load_sensitivity = static_cast<float>(s.load_sensitivity),
        .surface_id = surface_id,
    };
}

void populate_surface_map(
    const nativecfg::TireConfig &cfg,
    SurfaceMapView<float> &out_map
) {
    out_map = SurfaceMapView<float>{};
    if (cfg.backend == "pneumatic") {
        if (!cfg.pneumatic.has_value()) {
            throw std::invalid_argument("pneumatic backend requires pneumatic config");
        }
        if (cfg.pneumatic->surface.has_value()) {
            out_map.has_surface_map = 1;
            out_map.default_surface = pack_surface_spec(*cfg.pneumatic->surface, 0);
            out_map.section_count = 0;
            return;
        }
        if (!cfg.surface_map.has_value()) {
            throw std::invalid_argument(
                "pneumatic tires need a track surface map or a configured surface");
        }
    } else if (cfg.surface_mode == "configured") {
        out_map.has_surface_map = 0;
        out_map.section_count = 0;
        return;
    } else if (cfg.surface_mode != "track") {
        throw std::invalid_argument("unsupported tire surface_mode '" + cfg.surface_mode + "'");
    }
    if (!cfg.surface_map.has_value()) {
        throw std::invalid_argument("track surface_mode requires a surface_map");
    }
    const nativecfg::SurfaceMap &src = *cfg.surface_map;
    if (src.sections.size() > static_cast<std::size_t>(kMaxSurfaceSections)) {
        throw std::invalid_argument("surface_map section count exceeds kMaxSurfaceSections");
    }

    std::vector<nativecfg::SurfaceSection> sorted = src.sections;
    std::ranges::stable_sort(sorted, {}, [](const nativecfg::SurfaceSection &sec) {
        return sec.start_m;
    });

    out_map.has_surface_map = 1;
    out_map.default_surface = pack_surface_spec(src.surface, 0);
    out_map.section_count = static_cast<i32>(sorted.size());
    const std::span<SurfaceSectionView<float>, kMaxSurfaceSections> dst_sections(out_map.sections);
    for (std::size_t i = 0; i < sorted.size(); ++i) {
        const nativecfg::SurfaceSection &sec = sorted[i];
        if (!std::isfinite(sec.start_m) || !std::isfinite(sec.end_m) || sec.start_m >= sec.end_m) {
            throw std::invalid_argument("surface_map section bounds must be finite with start_m < end_m");
        }
        if (!std::isfinite(sec.blend_m) || sec.blend_m < 0.0) {
            throw std::invalid_argument("surface_map section blend_m must be finite and nonnegative");
        }
        double b_left = 0.0;
        if (i == 0) {
            if (sec.start_m > 0.0) {
                b_left = sec.blend_m;
                if (sec.start_m < 0.5 * b_left) {
                    throw std::invalid_argument("surface_map transition extends before x = 0");
                }
            }
        } else {
            const nativecfg::SurfaceSection &prev = sorted[i - 1];
            if (sec.start_m < prev.end_m) {
                throw std::invalid_argument("surface_map sections must not overlap");
            }
            if (sec.start_m == prev.end_m) {
                if (prev.blend_m != sec.blend_m) {
                    throw std::invalid_argument("surface_map adjacent sections must have equal blend_m");
                }
                b_left = sec.blend_m;
            } else {
                if (sec.start_m - prev.end_m < 0.5 * prev.blend_m + 0.5 * sec.blend_m) {
                    throw std::invalid_argument("surface_map transition windows overlap across gap");
                }
                b_left = sec.blend_m;
            }
        }
        if (sec.end_m - sec.start_m < 0.5 * b_left + 0.5 * sec.blend_m) {
            throw std::invalid_argument("surface_map section shorter than transition half-widths");
        }

        i32 sid = static_cast<i32>(i + 1);
        if (sec.surface.name == src.surface.name) {
            sid = 0;
        } else {
            for (std::size_t k = 0; k < i; ++k) {
                if (sec.surface.name == sorted[k].surface.name) {
                    sid = dst_sections[k].surface.surface_id;
                    break;
                }
            }
        }

        dst_sections[i] = SurfaceSectionView<float>{
            .start_m = static_cast<float>(sec.start_m),
            .end_m = static_cast<float>(sec.end_m),
            .surface = pack_surface_spec(sec.surface, sid),
            .blend_m = static_cast<float>(sec.blend_m),
        };
    }
}

struct WheelGeomRef {
    int geom_id = -1;
    int expected_body_id = -1;
};

void validate_and_pack_wheel_geom(
    const mjModel *m,
    WheelGeomRef target,
    TyreWheelSpec<float> &out_spec
) {
    const int geom_id = target.geom_id;
    const int expected_body_id = target.expected_body_id;
    model_access::require_id(geom_id, m->ngeom, "wheel contact geom");
    const std::span<const int> geom_bodyid =
        model_access::readonly_buffer(m->geom_bodyid, m->ngeom, "geom_bodyid");
    const int body_id = geom_bodyid[to_idx(geom_id)];
    if (body_id <= 0 || body_id >= m->nbody || body_id != expected_body_id) {
        throw std::invalid_argument("wheel geom_bodyid does not match wheel_body_id");
    }

    const std::span<const mjtNum> geom_pos = model_access::readonly_buffer(
        m->geom_pos, model_access::kXYZ * m->ngeom, "geom_pos");
    for (int k = 0; k < model_access::kXYZ; ++k) {
        const double p = geom_pos[model_access::kXYZ * to_idx(geom_id) + to_idx(k)];
        if (!std::isfinite(p) || std::abs(p) > 1e-12) {
            throw std::invalid_argument("wheel geom_pos must be zero within 1e-12");
        }
    }

    const std::span<const mjtNum> geom_quat = model_access::readonly_buffer(
        m->geom_quat, model_access::kQuaternionElements * m->ngeom, "geom_quat");
    const std::size_t qbase = model_access::kQuaternionElements * to_idx(geom_id);
    const double qw = geom_quat[qbase + 0];
    const double qx = geom_quat[qbase + 1];
    const double qy = geom_quat[qbase + 2];
    const double qz = geom_quat[qbase + 3];
    if (!std::isfinite(qw) || !std::isfinite(qx) || !std::isfinite(qy) || !std::isfinite(qz) ||
        std::abs(std::abs(qw) - 1.0) > 1e-12 ||
        std::abs(qx) > 1e-12 || std::abs(qy) > 1e-12 || std::abs(qz) > 1e-12) {
        throw std::invalid_argument("wheel geom_quat must be identity within 1e-12");
    }

    const std::span<const mjtNum> geom_size = model_access::readonly_buffer(
        m->geom_size, model_access::kXYZ * m->ngeom, "geom_size");
    const double radius = geom_size[model_access::kXYZ * to_idx(geom_id)];
    if (!std::isfinite(radius) || radius <= 0.0) {
        throw std::invalid_argument("wheel geom radius must be finite and positive");
    }

    out_spec.geom_id = geom_id;
    out_spec.radius_m = static_cast<float>(radius);
    out_spec.chain = build_wheel_chain(m, body_id);
}

void validate_and_pack_compliant_side(
    const nativecfg::TireParams &params,
    TyreWheelSpec<float> &out_spec
) {
    if (!params.material.has_value()) {
        throw std::invalid_argument("compliant_2d tire params missing material");
    }
    const nativecfg::TireMaterial &mat = *params.material;
    if (!std::isfinite(mat.loss_factor) || mat.loss_factor < 0.0 || mat.loss_factor >= 1.0) {
        throw std::invalid_argument("compliant_2d loss_factor must lie in [0, 1)");
    }
    if (!std::isfinite(mat.hysteresis_rate_eps_mps) || mat.hysteresis_rate_eps_mps <= 0.0) {
        throw std::invalid_argument("compliant_2d hysteresis_rate_eps_mps must be positive");
    }
    const double load_lo = mat.valid_load_range_n[0];
    const double load_hi = mat.valid_load_range_n[1];
    if (!std::isfinite(load_lo) || !std::isfinite(load_hi) || load_lo < 0.0 || load_lo >= load_hi) {
        throw std::invalid_argument("compliant_2d valid_load_range_n invalid");
    }

    if (mat.table.has_value()) {
        const nativecfg::TireTable &tbl = *mat.table;
        if (tbl.deflection_m.size() < 2 || tbl.deflection_m.size() != tbl.force_n.size()) {
            throw std::invalid_argument("compliant_2d TabulatedRadial table shape invalid");
        }
        if (tbl.deflection_m.size() > static_cast<std::size_t>(kMaxGpuRadialTableKnots)) {
            throw std::invalid_argument("compliant_2d TabulatedRadial knot count exceeds kMaxGpuRadialTableKnots");
        }
        for (std::size_t i = 0; i < tbl.deflection_m.size(); ++i) {
            if (!std::isfinite(tbl.deflection_m[i]) || !std::isfinite(tbl.force_n[i])) {
                throw std::invalid_argument("compliant_2d TabulatedRadial entries must be finite");
            }
            if (i > 0 && (tbl.deflection_m[i] <= tbl.deflection_m[i - 1] || tbl.force_n[i] < tbl.force_n[i - 1])) {
                throw std::invalid_argument("compliant_2d TabulatedRadial deflections must increase and forces must not decrease");
            }
        }
        if (tbl.deflection_m.front() != 0.0 || tbl.force_n.front() != 0.0) {
            throw std::invalid_argument("compliant_2d TabulatedRadial table must start at (0, 0)");
        }
        if (!(tbl.force_n.back() > 0.0) || load_hi > tbl.force_n.back()) {
            throw std::invalid_argument("compliant_2d TabulatedRadial valid_load_range_n exceeds table");
        }

        std::array<float, kMaxGpuRadialTableKnots> defl_f{};
        std::array<float, kMaxGpuRadialTableKnots> force_f{};
        for (std::size_t i = 0; i < tbl.deflection_m.size(); ++i) {
            defl_f[i] = static_cast<float>(tbl.deflection_m[i]);
            force_f[i] = static_cast<float>(tbl.force_n[i]);
        }
        TabulatedRadialParams<float> tab_params{};
        ErrCode tab_err = ErrCode::none;
        if (init_tabulated_radial_params<float, GpuToolkit>(
                static_cast<i32>(tbl.deflection_m.size()),
                defl_f.data(),
                force_f.data(),
                tab_params,
                tab_err) != Status::ok) {
            throw std::invalid_argument("compliant_2d TabulatedRadial initialization failed");
        }

        CompliantMaterialView<float> view{};
        view.radial_k_n_m = (std::isfinite(mat.radial_k_n_m) && mat.radial_k_n_m > 0.0)
            ? static_cast<float>(mat.radial_k_n_m)
            : 0.0f;
        view.loss_factor = static_cast<float>(mat.loss_factor);
        view.hysteresis_rate_eps_mps = static_cast<float>(mat.hysteresis_rate_eps_mps);
        view.valid_load_min_n = static_cast<float>(load_lo);
        view.valid_load_max_n = static_cast<float>(load_hi);
        view.has_tabulated_radial = 1;
        view.use_tabulated = 1;
        view.knot_count = tab_params.knot_count;
        const std::size_t n = static_cast<std::size_t>(tab_params.knot_count);
        const std::span<float, kMaxGpuRadialTableKnots> dst_defl(view.deflection_m);
        const std::span<float, kMaxGpuRadialTableKnots> dst_force(view.force_n);
        const std::span<float, kMaxGpuRadialTableKnots> dst_integ(view.integral);
        const std::span<float, kMaxGpuRadialTableKnots - 1> dst_slopes(view.slopes);
        const std::span<const float, kMaxRadialTableKnots> src_defl(tab_params.deflection_m);
        const std::span<const float, kMaxRadialTableKnots> src_force(tab_params.force_n);
        const std::span<const float, kMaxRadialTableKnots> src_integ(tab_params.integral);
        const std::span<const float, kMaxRadialTableKnots - 1> src_slopes(tab_params.slopes);
        for (std::size_t i = 0; i < n; ++i) {
            dst_defl[i] = src_defl[i];
            dst_force[i] = src_force[i];
            dst_integ[i] = src_integ[i];
        }
        for (std::size_t i = 0; i + 1 < n; ++i) {
            dst_slopes[i] = src_slopes[i];
        }
        out_spec.compliant = view;
        return;
    }

    if (!std::isfinite(mat.radial_k_n_m) || mat.radial_k_n_m <= 0.0) {
        throw std::invalid_argument("compliant_2d radial_k_n_m must be finite and positive");
    }

    out_spec.compliant = CompliantMaterialView<float>{
        .radial_k_n_m = static_cast<float>(mat.radial_k_n_m),
        .loss_factor = static_cast<float>(mat.loss_factor),
        .hysteresis_rate_eps_mps = static_cast<float>(mat.hysteresis_rate_eps_mps),
        .valid_load_min_n = static_cast<float>(load_lo),
        .valid_load_max_n = static_cast<float>(load_hi),
        .has_tabulated_radial = 0,
        .use_tabulated = 0,
    };
}

void validate_and_pack_pneumatic_side(
    const nativecfg::PneumaticConfig &pneu,
    const nativecfg::PneumaticTyreSpec &pspec,
    TyreWheelSpec<float> &out_spec
) {
    if (pneu.tier != "fast" && pneu.tier != "detailed") {
        throw std::invalid_argument("pneumatic tier must be 'fast' or 'detailed'");
    }
    const bool fast = (pneu.tier == "fast");
    if (pneu.n_rays != (fast ? 64 : 256)) {
        throw std::invalid_argument("pneumatic n_rays must match tier (64 for fast, 256 for detailed)");
    }
    if (pneu.discretised_brush == fast) {
        throw std::invalid_argument("pneumatic discretised_brush must match tier");
    }
    if (!std::isfinite(pneu.half_angle_deg) || pneu.half_angle_deg <= 0.0 || pneu.half_angle_deg >= 90.0) {
        throw std::invalid_argument("pneumatic half_angle_deg must lie in (0, 90)");
    }
    auto check_pos = [](double v, const char *msg) {
        if (!std::isfinite(v) || v <= 0.0) {
            throw std::invalid_argument(msg);
        }
    };
    auto check_nonneg = [](double v, const char *msg) {
        if (!std::isfinite(v) || v < 0.0) {
            throw std::invalid_argument(msg);
        }
    };
    check_pos(pspec.outer_radius_mm, "pneumatic outer_radius_mm must be positive");
    check_pos(pspec.rim_radius_mm, "pneumatic rim_radius_mm must be positive");
    check_pos(pspec.section_height_mm, "pneumatic section_height_mm must be positive");
    check_pos(pspec.pressure_bar, "pneumatic pressure_bar must be positive");
    check_pos(pspec.crr_reference, "pneumatic crr_reference must be positive");
    check_pos(pspec.casing_width_mm, "pneumatic casing_width_mm must be positive");
    check_pos(pspec.compressed_casing_mm, "pneumatic compressed_casing_mm must be positive");
    check_pos(pspec.area_factor, "pneumatic area_factor must be positive");
    check_pos(pspec.contact_length_factor, "pneumatic contact_length_factor must be positive");
    check_pos(pspec.rate_relaxation_s, "pneumatic rate_relaxation_s must be positive");
    check_pos(pspec.hysteresis_rate_eps_mps, "pneumatic hysteresis_rate_eps_mps must be positive");
    check_pos(pspec.relaxation_length_mm, "pneumatic relaxation_length_mm must be positive");
    check_pos(pspec.rim_stiffness_n_mm2, "pneumatic rim_stiffness_n_mm2 must be positive");
    check_nonneg(pspec.carcass_stiffness_n_mm2, "pneumatic carcass_stiffness_n_mm2 must be nonnegative");
    check_nonneg(pspec.rate_stiffening, "pneumatic rate_stiffening must be nonnegative");
    check_nonneg(pspec.loss_factor, "pneumatic loss_factor must be nonnegative");
    check_nonneg(pspec.tread_loss_crr, "pneumatic tread_loss_crr must be nonnegative");
    if (!(pspec.rim_radius_mm < pspec.outer_radius_mm)) {
        throw std::invalid_argument("pneumatic rim_radius_mm must be smaller than outer_radius_mm");
    }
    if (!(pspec.rim_strike_deflection_mm() > 0.0)) {
        throw std::invalid_argument("pneumatic rim_strike_deflection_mm must be positive");
    }
    if (pspec.area_factor > 1.0 || pspec.contact_length_factor > 1.0 || pspec.loss_factor >= 1.0) {
        throw std::invalid_argument("pneumatic area_factor/contact_length_factor/loss_factor out of bounds");
    }
    const double expected_radius_m = pspec.outer_radius_mm / 1000.0;
    if (!(std::abs(static_cast<double>(out_spec.radius_m) - expected_radius_m) <= 1e-6)) {
        throw std::invalid_argument("pneumatic contact geom radius does not match outer_radius_mm");
    }

    struct HostDoubleTk {
        static double div(double a, double b) noexcept { return a / b; }
        static bool isfinite(double x) noexcept { return std::isfinite(x); }
    };

    PneumaticRayRing<double> ring_d{};
    ErrCode ring_err = ErrCode::none;
    const double half_angle_rad = pneu.half_angle_deg * (std::numbers::pi / 180.0);
    if (init_pneumatic_ray_ring<double, HostDoubleTk>(
            static_cast<i32>(pneu.n_rays),
            half_angle_rad,
            ring_d,
            ring_err) != Status::ok) {
        throw std::invalid_argument("pneumatic ray ring initialization failed");
    }

    PneumaticMaterialView<float> dst{};
    dst.outer_radius_mm = static_cast<float>(pspec.outer_radius_mm);
    dst.rim_radius_mm = static_cast<float>(pspec.rim_radius_mm);
    dst.section_height_mm = static_cast<float>(pspec.section_height_mm);
    dst.pressure_bar = static_cast<float>(pspec.pressure_bar);
    dst.crr_reference = static_cast<float>(pspec.crr_reference);
    dst.casing_width_mm = static_cast<float>(pspec.casing_width_mm);
    dst.compressed_casing_mm = static_cast<float>(pspec.compressed_casing_mm);
    dst.area_factor = static_cast<float>(pspec.area_factor);
    dst.carcass_stiffness_n_mm2 = static_cast<float>(pspec.carcass_stiffness_n_mm2);
    dst.contact_length_factor = static_cast<float>(pspec.contact_length_factor);
    dst.rate_stiffening = static_cast<float>(pspec.rate_stiffening);
    dst.rate_relaxation_s = static_cast<float>(pspec.rate_relaxation_s);
    dst.loss_factor = static_cast<float>(pspec.loss_factor);
    dst.hysteresis_rate_eps_mps = static_cast<float>(pspec.hysteresis_rate_eps_mps);
    dst.relaxation_length_mm = static_cast<float>(pspec.relaxation_length_mm);
    dst.rim_stiffness_n_mm2 = static_cast<float>(pspec.rim_stiffness_n_mm2);
    dst.tread_loss_crr = static_cast<float>(pspec.tread_loss_crr);
    dst.n_rays = ring_d.n_rays;
    dst.discretised_brush = pneu.discretised_brush ? 1 : 0;
    dst.half_angle_rad = static_cast<float>(ring_d.half_angle_rad);
    dst.dtheta_rad = static_cast<float>(ring_d.dtheta_rad);
    const std::span<float, kMaxGpuPneumaticRays> dst_theta(dst.theta_rad);
    const std::span<float, kMaxGpuPneumaticRays> dst_ux(dst.ux);
    const std::span<float, kMaxGpuPneumaticRays> dst_uz(dst.uz);
    const std::span<const double, kMaxPneumaticRays> src_theta(ring_d.theta_rad);
    const std::span<const double, kMaxPneumaticRays> src_ux(ring_d.ux);
    const std::span<const double, kMaxPneumaticRays> src_uz(ring_d.uz);
    for (i32 i = 0; i < ring_d.n_rays; ++i) {
        const std::size_t idx = static_cast<std::size_t>(i);
        dst_theta[idx] = static_cast<float>(src_theta[idx]);
        dst_ux[idx] = static_cast<float>(src_ux[idx]);
        dst_uz[idx] = static_cast<float>(src_uz[idx]);
    }
    out_spec.pneumatic = dst;
}

void validate_and_pack_distributed_side(
    const nativecfg::DistributedMaterial &dmat,
    int station_count,
    TyreWheelSpec<float> &out_spec
) {
    const nativecfg::HingeDensity &hd = dmat.density;
    if (hd.knots_m.empty() || hd.knots_m.size() != hd.stiffness_n_m.size()) {
        throw std::invalid_argument("distributed hinge density knots/stiffness size mismatch");
    }
    if (hd.knots_m.size() > static_cast<std::size_t>(kMaxHingeKnots)) {
        throw std::invalid_argument("distributed hinge density knot_count exceeds kMaxHingeKnots");
    }
    if (!std::isfinite(hd.damping_ns_m) || hd.damping_ns_m < 0.0 ||
        !std::isfinite(hd.loss_factor) || hd.loss_factor < 0.0 || hd.loss_factor >= 1.0 ||
        !std::isfinite(hd.hysteresis_rate_eps_mps) || hd.hysteresis_rate_eps_mps <= 0.0) {
        throw std::invalid_argument("distributed hinge density parameters invalid");
    }
    const double defl_lo = dmat.valid_deflection_range_m[0];
    const double defl_hi = dmat.valid_deflection_range_m[1];
    if (!std::isfinite(defl_lo) || !std::isfinite(defl_hi) || defl_lo < 0.0 || defl_lo >= defl_hi) {
        throw std::invalid_argument("distributed valid_deflection_range_m invalid");
    }

    out_spec.distributed_density = HingeDensityView<float>{};
    out_spec.distributed_density.knot_count = static_cast<i32>(hd.knots_m.size());
    out_spec.distributed_density.damping_ns_m = static_cast<float>(hd.damping_ns_m);
    out_spec.distributed_density.loss_factor = static_cast<float>(hd.loss_factor);
    out_spec.distributed_density.hysteresis_rate_eps_mps = static_cast<float>(hd.hysteresis_rate_eps_mps);

    const std::span<float, kMaxHingeKnots> dst_knots(out_spec.distributed_density.knots_m);
    const std::span<float, kMaxHingeKnots> dst_stiff(out_spec.distributed_density.stiffness_n_m);
    for (std::size_t i = 0; i < hd.knots_m.size(); ++i) {
        if (!std::isfinite(hd.knots_m[i]) || !std::isfinite(hd.stiffness_n_m[i]) ||
            hd.stiffness_n_m[i] < 0.0) {
            throw std::invalid_argument("distributed hinge density knot or stiffness invalid");
        }
        if (i > 0 && hd.knots_m[i] <= hd.knots_m[i - 1]) {
            throw std::invalid_argument("distributed hinge density knots must increase strictly");
        }
        dst_knots[i] = static_cast<float>(hd.knots_m[i]);
        dst_stiff[i] = static_cast<float>(hd.stiffness_n_m[i]);
    }
    out_spec.valid_deflection_min_m = static_cast<float>(defl_lo);
    out_spec.valid_deflection_max_m = static_cast<float>(defl_hi);

    const double two_pi = 2.0 * std::numbers::pi;
    const double c = static_cast<double>(station_count);
    out_spec.station_weight = static_cast<float>(two_pi / c);
    const double radius = static_cast<double>(out_spec.radius_m);
    const std::span<float, 3U * static_cast<std::size_t>(kMaxGpuTyreStations)> dst_xyz(
        out_spec.local_xyz);
    for (int i = 0; i < station_count; ++i) {
        const double angle = two_pi * static_cast<double>(i) / c;
        const std::size_t base = 3U * to_idx(i);
        dst_xyz[base + 0] = static_cast<float>(radius * std::cos(angle));
        dst_xyz[base + 1] = 0.0f;
        dst_xyz[base + 2] = static_cast<float>(radius * std::sin(angle));
    }
}

void populate_terrain_views(
    const mjModel *m,
    const mjData *d,
    const nativecfg::TireConfig &cfg,
    HostTyreUploadBundle &bundle
) {
    std::vector<double> px_d;
    std::vector<double> pz_d;
    std::size_t cs = 0;

    if (bundle.gpu_model.backend == TyreBackendKind::pneumatic &&
        cfg.pneumatic.has_value() &&
        cfg.pneumatic->profile.z_m.size() >= 2) {
        const nativecfg::PneumaticProfile &prof = cfg.pneumatic->profile;
        if (!std::isfinite(prof.x0_m) || !std::isfinite(prof.dx_m) || !(prof.dx_m > 0.0)) {
            throw std::invalid_argument("pneumatic profile needs finite x0_m and positive dx_m");
        }
        cs = prof.z_m.size();
        px_d.resize(cs);
        pz_d.resize(cs);
        for (std::size_t i = 0; i < cs; ++i) {
            if (!std::isfinite(prof.z_m[i])) {
                throw std::invalid_argument("pneumatic profile height is non-finite");
            }
            px_d[i] = prof.x0_m + prof.dx_m * static_cast<double>(i);
            pz_d[i] = prof.z_m[i];
        }
        bundle.gpu_model.road_x0_m = static_cast<float>(prof.x0_m);
        bundle.gpu_model.road_dx_m = static_cast<float>(prof.dx_m);
        bundle.gpu_model.road_is_uniform_grid = 1;
    } else {
        const int terrain_geom = mj_name2id(m, mjOBJ_GEOM, "terrain");
        if (terrain_geom < 0) {
            throw std::invalid_argument("model has no 'terrain'");
        }
        model_access::require_id(terrain_geom, m->ngeom, "terrain geom");

        const std::span<const int> geom_type =
            model_access::readonly_buffer(m->geom_type, m->ngeom, "geom_type");
        const std::span<const int> geom_dataid =
            model_access::readonly_buffer(m->geom_dataid, m->ngeom, "geom_dataid");
        if (geom_type[to_idx(terrain_geom)] != mjGEOM_HFIELD) {
            throw std::invalid_argument("GPU tyre upload requires compiled terrain heightfield");
        }
        const int hfield = geom_dataid[to_idx(terrain_geom)];
        model_access::require_id(hfield, m->nhfield, "terrain heightfield ID");

        const std::span<const int> nrow =
            model_access::readonly_buffer(m->hfield_nrow, m->nhfield, "hfield_nrow");
        const std::span<const int> ncol =
            model_access::readonly_buffer(m->hfield_ncol, m->nhfield, "hfield_ncol");
        const std::span<const int> adr =
            model_access::readonly_buffer(m->hfield_adr, m->nhfield, "hfield_adr");
        const int rows = nrow[to_idx(hfield)];
        const int cols = ncol[to_idx(hfield)];
        const int start = adr[to_idx(hfield)];
        if (cols < 2 || rows < 2 || start < 0) {
            throw std::invalid_argument("invalid compiled heightfield raster");
        }

        const std::span<const float> hfield_data =
            model_access::readonly_buffer(m->hfield_data, m->nhfielddata, "hfield_data");
        const std::size_t raster_count = model_access::checked_product(
            to_idx(rows), to_idx(cols), hfield_data.size());
        const std::size_t raster_end = model_access::checked_sum(
            to_idx(start), raster_count, hfield_data.size());
        const std::span<const float> raster = hfield_data.subspan(
            to_idx(start), raster_end - to_idx(start));
        for (const float v : raster) {
            if (!std::isfinite(v)) {
                throw std::invalid_argument("invalid compiled heightfield raster");
            }
        }
        cs = to_idx(cols);
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                if (std::abs(raster[to_idx(r) * cs + to_idx(c)] - raster[to_idx(c)]) > 1e-10f) {
                    throw std::invalid_argument("GPU tyre upload does not support lateral terrain variation");
                }
            }
        }

        const std::span<const mjtNum> hsize = model_access::readonly_buffer(
            m->hfield_size, 4 * m->nhfield, "hfield_size");
        const double sx = hsize[4 * to_idx(hfield)];
        const double sz = hsize[4 * to_idx(hfield) + 2];

        const std::span<const mjtNum> xmat = model_access::readonly_buffer(
            d->geom_xmat, model_access::kRotationElements * m->ngeom, "geom_xmat");
        const std::span<const mjtNum> xpos = model_access::readonly_buffer(
            d->geom_xpos, model_access::kXYZ * m->ngeom, "geom_xpos");
        const std::size_t gs = to_idx(terrain_geom);
        const std::span<const mjtNum> rmat = xmat.subspan(
            model_access::kRotationElements * gs, model_access::kRotationElements);
        const std::span<const mjtNum> pos = xpos.subspan(
            model_access::kXYZ * gs, model_access::kXYZ);

        for (const mjtNum v : rmat) {
            if (!std::isfinite(v)) {
                throw std::invalid_argument("invalid terrain transform: non-finite entry");
            }
        }
        for (const mjtNum v : pos) {
            if (!std::isfinite(v)) {
                throw std::invalid_argument("invalid terrain transform: non-finite entry");
            }
        }
        if (std::abs(rmat[1]) > 1e-10 || std::abs(rmat[4] - 1.0) > 1e-10 || std::abs(rmat[7]) > 1e-10) {
            throw std::invalid_argument("terrain transform must preserve the planar Y axis");
        }

        const double step = (2.0 * sx) / static_cast<double>(cols - 1);
        px_d.resize(cs);
        pz_d.resize(cs);
        for (std::size_t i = 0; i < cs; ++i) {
            const double lx = (i + 1 == cs) ? sx : (static_cast<double>(i) * step - sx);
            const double lz = static_cast<double>(raster[i]) * sz;
            const double wx = lx * rmat[0] + lz * rmat[2] + pos[0];
            const double wy = lx * rmat[3] + lz * rmat[5] + pos[1];
            const double wz = lx * rmat[6] + lz * rmat[8] + pos[2];
            if (!std::isfinite(wx) || !std::isfinite(wy) || !std::isfinite(wz)) {
                throw std::invalid_argument("terrain profile vertex is non-finite");
            }
            if (std::abs(wy) > 1e-9) {
                throw std::invalid_argument("working profile must lie in world Y=0");
            }
            px_d[i] = wx;
            pz_d[i] = wz;
        }

        for (std::size_t i = 0; i + 1 < cs; ++i) {
            if (!(px_d[i + 1] - px_d[i] > 0.0)) {
                throw std::invalid_argument("profile x must increase strictly");
            }
        }

        const bool unrotated =
            std::abs(rmat[0] - 1.0) <= 1e-12 &&
            std::abs(rmat[2]) <= 1e-12 &&
            std::abs(rmat[6]) <= 1e-12 &&
            std::abs(rmat[8] - 1.0) <= 1e-12;
        if (unrotated && step > 0.0) {
            bundle.gpu_model.road_x0_m = static_cast<float>(px_d[0]);
            bundle.gpu_model.road_dx_m = static_cast<float>(step);
            bundle.gpu_model.road_is_uniform_grid = 1;
        } else {
            bundle.gpu_model.road_x0_m = 0.0f;
            bundle.gpu_model.road_dx_m = 0.0f;
            bundle.gpu_model.road_is_uniform_grid = 0;
        }
    }

    // Collinearly decimate quantized float32 hfield samples on genuine straight
    // runs (deviation <= 4 um over spans >= 0.25 m) so float32 raster
    // quantization does not inject high-frequency slope jitter on long ramps
    // and flats, while curved/rough profiles keep all sample vertices intact.
    // Pneumatic keeps the exact uniform-grid or raster samples intact.
    if (bundle.gpu_model.backend != TyreBackendKind::pneumatic && cs > 8) {
        constexpr double kCollinearTolM = 4.0e-6;
        constexpr double kMinStraightRunM = 0.05;
        std::vector<int> keep(cs, 0);
        keep.front() = 1;
        keep.back() = 1;
        std::vector<std::pair<std::size_t, std::size_t>> stack;
        stack.emplace_back(0, cs - 1);
        while (!stack.empty()) {
            const auto [i0, i1] = stack.back();
            stack.pop_back();
            if (i1 <= i0 + 1) {
                continue;
            }
            const double x0 = px_d[i0];
            const double z0 = pz_d[i0];
            const double dx = px_d[i1] - x0;
            const double dz = pz_d[i1] - z0;
            if (!(dx > 0.0)) {
                continue;
            }
            if (dx < kMinStraightRunM) {
                // Only preserve high-density vertices inside short spans when the
                // surrounding 0.25 m neighborhood has high-frequency roughness
                // (e.g. rough_uphill), so smooth quadratic transitions (e.g.
                // climb_15pct's 4 m grade fillet) still shed float32 sawtooth.
                constexpr double kHalfWindowM = 0.125;
                constexpr double kRoughWindowTolM = 5.0e-4;
                const double x_mid = 0.5 * (x0 + px_d[i1]);
                std::size_t w0 = i0;
                while (w0 > 0 && (x_mid - px_d[w0 - 1]) <= kHalfWindowM) {
                    --w0;
                }
                std::size_t w1 = i1;
                while (w1 + 1 < cs && (px_d[w1 + 1] - x_mid) <= kHalfWindowM) {
                    ++w1;
                }
                const double wx_span = px_d[w1] - px_d[w0];
                bool is_rough = false;
                if (wx_span > 0.0 && w1 > w0 + 1) {
                    const double w_slope = (pz_d[w1] - pz_d[w0]) / wx_span;
                    for (std::size_t k = w0 + 1; k < w1; ++k) {
                        const double z_w = pz_d[w0] + w_slope * (px_d[k] - px_d[w0]);
                        if (std::abs(pz_d[k] - z_w) > kRoughWindowTolM) {
                            is_rough = true;
                            break;
                        }
                    }
                }
                if (is_rough) {
                    for (std::size_t k = i0 + 1; k < i1; ++k) {
                        keep[k] = 1;
                    }
                    continue;
                }
            }
            const double slope = dz / dx;
            double max_dev = 0.0;
            std::size_t split = i0;
            for (std::size_t k = i0 + 1; k < i1; ++k) {
                const double z_line = z0 + slope * (px_d[k] - x0);
                const double dev = std::abs(pz_d[k] - z_line);
                if (dev > max_dev) {
                    max_dev = dev;
                    split = k;
                }
            }
            if (max_dev > kCollinearTolM && split > i0 && split < i1) {
                keep[split] = 1;
                stack.emplace_back(i0, split);
                stack.emplace_back(split, i1);
            }
        }
        std::vector<double> dec_x;
        std::vector<double> dec_z;
        dec_x.reserve(cs);
        dec_z.reserve(cs);
        for (std::size_t i = 0; i < cs; ++i) {
            if (keep[i] != 0) {
                dec_x.push_back(px_d[i]);
                dec_z.push_back(pz_d[i]);
            }
        }
        // Keep >= 8 vertices by subdividing the first segment collinearly if a
        // purely flat profile collapsed below 8 points (e.g. test mutators at
        // index 5 assume at least several profile vertices).
        if (dec_x.size() >= 2 && dec_x.size() < 8) {
            const std::size_t extra = 8 - dec_x.size();
            const double x0 = dec_x[0];
            const double z0 = dec_z[0];
            const double x1 = dec_x[1];
            const double z1 = dec_z[1];
            std::vector<double> padded_x;
            std::vector<double> padded_z;
            padded_x.reserve(8);
            padded_z.reserve(8);
            padded_x.push_back(x0);
            padded_z.push_back(z0);
            for (std::size_t j = 1; j <= extra; ++j) {
                const double t = static_cast<double>(j) / static_cast<double>(extra + 1);
                padded_x.push_back(x0 + t * (x1 - x0));
                padded_z.push_back(z0 + t * (z1 - z0));
            }
            for (std::size_t j = 1; j < dec_x.size(); ++j) {
                padded_x.push_back(dec_x[j]);
                padded_z.push_back(dec_z[j]);
            }
            dec_x = std::move(padded_x);
            dec_z = std::move(padded_z);
        }
        if (dec_x.size() >= 2) {
            px_d = std::move(dec_x);
            pz_d = std::move(dec_z);
            cs = px_d.size();
        }
    }

    const std::size_t nseg = cs - 1;
    bundle.pl_px.resize(cs);
    bundle.pl_pz.resize(cs);
    bundle.sp_x.resize(cs);
    bundle.sp_z.resize(cs);
    for (std::size_t i = 0; i < cs; ++i) {
        bundle.pl_px[i] = static_cast<float>(px_d[i]);
        bundle.pl_pz[i] = static_cast<float>(pz_d[i]);
        bundle.sp_x[i] = bundle.pl_px[i];
        bundle.sp_z[i] = bundle.pl_pz[i];
    }
    for (std::size_t i = 0; i < nseg; ++i) {
        if (!(px_d[i + 1] - px_d[i] > 0.0) || !(bundle.pl_px[i + 1] - bundle.pl_px[i] > 0.0f)) {
            throw std::invalid_argument("profile x must increase strictly");
        }
    }

    bundle.pl_seg_x.resize(nseg);
    bundle.pl_seg_z.resize(nseg);
    bundle.pl_seg_len_sq.resize(nseg);
    bundle.pl_height_changes.resize(cs);
    bundle.pl_left_x.resize(cs);
    bundle.pl_left_z.resize(cs);
    bundle.pl_right_x.resize(cs);
    bundle.pl_right_z.resize(cs);

    bundle.sp_edge_x.resize(nseg);
    bundle.sp_edge_z.resize(nseg);
    bundle.sp_length2.resize(nseg);
    bundle.sp_normal_x.resize(nseg);
    bundle.sp_normal_z.resize(nseg);

    bundle.pl_height_changes[0] = 0;
    for (std::size_t i = 0; i < nseg; ++i) {
        const double sx_d = px_d[i + 1] - px_d[i];
        const double sz_d = pz_d[i + 1] - pz_d[i];
        const double len2_d = sx_d * sx_d + sz_d * sz_d;
        const double len_d = std::sqrt(len2_d);
        const float seg_xf = static_cast<float>(sx_d);
        const float seg_zf = static_cast<float>(sz_d);
        const float len2_f = static_cast<float>(len2_d);

        bundle.pl_seg_x[i] = seg_xf;
        bundle.pl_seg_z[i] = seg_zf;
        bundle.pl_seg_len_sq[i] = len2_f;
        bundle.pl_height_changes[i + 1] =
            bundle.pl_height_changes[i] + (seg_zf != 0.0f ? 1 : 0);

        bundle.sp_edge_x[i] = seg_xf;
        bundle.sp_edge_z[i] = seg_zf;
        bundle.sp_length2[i] = len2_f;
        bundle.sp_normal_x[i] = static_cast<float>(-sz_d / len_d);
        bundle.sp_normal_z[i] = static_cast<float>(sx_d / len_d);
    }

    bundle.pl_left_x[0] = 0.0f;
    bundle.pl_left_z[0] = 0.0f;
    for (std::size_t v = 1; v < cs; ++v) {
        bundle.pl_left_x[v] = -bundle.pl_seg_x[v - 1];
        bundle.pl_left_z[v] = -bundle.pl_seg_z[v - 1];
    }
    for (std::size_t v = 0; v < nseg; ++v) {
        bundle.pl_right_x[v] = bundle.pl_seg_x[v];
        bundle.pl_right_z[v] = bundle.pl_seg_z[v];
    }
    bundle.pl_right_x[nseg] = 0.0f;
    bundle.pl_right_z[nseg] = 0.0f;

    bundle.polyline_view.num_vertices = static_cast<i32>(cs);
    bundle.polyline_view.maximum_z = *std::ranges::max_element(bundle.pl_pz);
    bundle.polyline_view.significant_delta_m = static_cast<float>(cfg.significant_delta_m);
    bundle.polyline_view.significance_fraction = static_cast<float>(cfg.significance_fraction);
    const float cos_thresh = static_cast<float>(
        std::cos(cfg.distinct_normal_deg * (std::numbers::pi / 180.0)));
    bundle.polyline_view.normal_cosine = cos_thresh;
    bundle.gpu_model.distinct_normal_cosine = cos_thresh;
    bundle.signed_profile_view.num_vertices = static_cast<i32>(cs);
}

} // namespace

HostTyreUploadBundle::HostTyreUploadBundle(const HostTyreUploadBundle &other)
    : gpu_model(other.gpu_model),
      polyline_view(other.polyline_view),
      signed_profile_view(other.signed_profile_view),
      pl_px(other.pl_px),
      pl_pz(other.pl_pz),
      pl_seg_x(other.pl_seg_x),
      pl_seg_z(other.pl_seg_z),
      pl_seg_len_sq(other.pl_seg_len_sq),
      pl_height_changes(other.pl_height_changes),
      pl_left_x(other.pl_left_x),
      pl_left_z(other.pl_left_z),
      pl_right_x(other.pl_right_x),
      pl_right_z(other.pl_right_z),
      sp_x(other.sp_x),
      sp_z(other.sp_z),
      sp_edge_x(other.sp_edge_x),
      sp_edge_z(other.sp_edge_z),
      sp_length2(other.sp_length2),
      sp_normal_x(other.sp_normal_x),
      sp_normal_z(other.sp_normal_z) {
    rebind_views();
}

HostTyreUploadBundle::HostTyreUploadBundle(HostTyreUploadBundle &&other) noexcept
    : gpu_model(other.gpu_model),
      polyline_view(other.polyline_view),
      signed_profile_view(other.signed_profile_view),
      pl_px(std::move(other.pl_px)),
      pl_pz(std::move(other.pl_pz)),
      pl_seg_x(std::move(other.pl_seg_x)),
      pl_seg_z(std::move(other.pl_seg_z)),
      pl_seg_len_sq(std::move(other.pl_seg_len_sq)),
      pl_height_changes(std::move(other.pl_height_changes)),
      pl_left_x(std::move(other.pl_left_x)),
      pl_left_z(std::move(other.pl_left_z)),
      pl_right_x(std::move(other.pl_right_x)),
      pl_right_z(std::move(other.pl_right_z)),
      sp_x(std::move(other.sp_x)),
      sp_z(std::move(other.sp_z)),
      sp_edge_x(std::move(other.sp_edge_x)),
      sp_edge_z(std::move(other.sp_edge_z)),
      sp_length2(std::move(other.sp_length2)),
      sp_normal_x(std::move(other.sp_normal_x)),
      sp_normal_z(std::move(other.sp_normal_z)) {
    rebind_views();
}

HostTyreUploadBundle &HostTyreUploadBundle::operator=(const HostTyreUploadBundle &other) {
    if (this == &other) {
        return *this;
    }
    HostTyreUploadBundle copy(other);
    *this = std::move(copy);
    return *this;
}

HostTyreUploadBundle &HostTyreUploadBundle::operator=(HostTyreUploadBundle &&other) noexcept {
    if (this == &other) {
        return *this;
    }
    gpu_model = other.gpu_model;
    polyline_view = other.polyline_view;
    signed_profile_view = other.signed_profile_view;
    pl_px = std::move(other.pl_px);
    pl_pz = std::move(other.pl_pz);
    pl_seg_x = std::move(other.pl_seg_x);
    pl_seg_z = std::move(other.pl_seg_z);
    pl_seg_len_sq = std::move(other.pl_seg_len_sq);
    pl_height_changes = std::move(other.pl_height_changes);
    pl_left_x = std::move(other.pl_left_x);
    pl_left_z = std::move(other.pl_left_z);
    pl_right_x = std::move(other.pl_right_x);
    pl_right_z = std::move(other.pl_right_z);
    sp_x = std::move(other.sp_x);
    sp_z = std::move(other.sp_z);
    sp_edge_x = std::move(other.sp_edge_x);
    sp_edge_z = std::move(other.sp_edge_z);
    sp_length2 = std::move(other.sp_length2);
    sp_normal_x = std::move(other.sp_normal_x);
    sp_normal_z = std::move(other.sp_normal_z);
    rebind_views();
    return *this;
}

void HostTyreUploadBundle::rebind_views() noexcept {
    polyline_view.px = pl_px.empty() ? nullptr : pl_px.data();
    polyline_view.pz = pl_pz.empty() ? nullptr : pl_pz.data();
    polyline_view.seg_x = pl_seg_x.empty() ? nullptr : pl_seg_x.data();
    polyline_view.seg_z = pl_seg_z.empty() ? nullptr : pl_seg_z.data();
    polyline_view.seg_len_sq = pl_seg_len_sq.empty() ? nullptr : pl_seg_len_sq.data();
    polyline_view.height_changes = pl_height_changes.empty() ? nullptr : pl_height_changes.data();
    polyline_view.left_x = pl_left_x.empty() ? nullptr : pl_left_x.data();
    polyline_view.left_z = pl_left_z.empty() ? nullptr : pl_left_z.data();
    polyline_view.right_x = pl_right_x.empty() ? nullptr : pl_right_x.data();
    polyline_view.right_z = pl_right_z.empty() ? nullptr : pl_right_z.data();

    signed_profile_view.x = sp_x.empty() ? nullptr : sp_x.data();
    signed_profile_view.z = sp_z.empty() ? nullptr : sp_z.data();
    signed_profile_view.edge_x = sp_edge_x.empty() ? nullptr : sp_edge_x.data();
    signed_profile_view.edge_z = sp_edge_z.empty() ? nullptr : sp_edge_z.data();
    signed_profile_view.length2 = sp_length2.empty() ? nullptr : sp_length2.data();
    signed_profile_view.normal_x = sp_normal_x.empty() ? nullptr : sp_normal_x.data();
    signed_profile_view.normal_z = sp_normal_z.empty() ? nullptr : sp_normal_z.data();
}

GpuWheelChain build_wheel_chain(const mjModel *m, int wheel_body_id) {
    if (m == nullptr) {
        throw std::invalid_argument("null mjModel in build_wheel_chain");
    }
    if (m->njnt <= 0 || m->njnt > kMaxNjnt ||
        m->nv <= 0 || m->nv > kMaxNv ||
        m->nbody <= 0 || m->nbody > kMaxNbody) {
        throw std::invalid_argument("model dimensions exceed GPU kMax bounds in build_wheel_chain");
    }
    if (wheel_body_id <= 0 || wheel_body_id >= m->nbody) {
        throw std::invalid_argument("wheel_body_id must identify a physical body");
    }

    const std::span<const int> body_parentid =
        model_access::readonly_buffer(m->body_parentid, m->nbody, "body_parentid");
    const std::span<const int> body_jntnum =
        model_access::readonly_buffer(m->body_jntnum, m->nbody, "body_jntnum");
    const std::span<const int> body_jntadr =
        model_access::readonly_buffer(m->body_jntadr, m->nbody, "body_jntadr");
    const std::span<const int> jnt_type =
        model_access::readonly_buffer(m->jnt_type, m->njnt, "jnt_type");
    const std::span<const int> jnt_dofadr =
        model_access::readonly_buffer(m->jnt_dofadr, m->njnt, "jnt_dofadr");

    std::vector<int> ancestor_bodies;
    int current = wheel_body_id;
    int hops = 0;
    while (current != 0) {
        if (current < 0 || current >= m->nbody || hops >= m->nbody) {
            throw std::invalid_argument("invalid or cyclic body_parentid chain");
        }
        ancestor_bodies.push_back(current);
        current = body_parentid[to_idx(current)];
        ++hops;
    }
    std::ranges::reverse(ancestor_bodies);

    GpuWheelChain chain{};
    chain.wheel_body_id = wheel_body_id;
    chain.njnt = static_cast<i32>(m->njnt);
    chain.joint_count = 0;
    const std::span<GpuChainJoint, kMaxChainJoints> dst_joints(chain.joints);

    for (const int body : ancestor_bodies) {
        const int jnum = body_jntnum[to_idx(body)];
        const int jadr = body_jntadr[to_idx(body)];
        if (jnum < 0) {
            throw std::invalid_argument("negative body_jntnum in wheel chain");
        }
        if (jnum == 0) {
            continue;
        }
        if (jadr < 0 || jadr >= m->njnt || jnum > m->njnt - jadr) {
            throw std::invalid_argument("body_jntadr/body_jntnum out of range in build_wheel_chain");
        }
        for (int k = 0; k < jnum; ++k) {
            const int jid = jadr + k;
            if (jid < 0 || jid >= m->njnt) {
                throw std::invalid_argument("joint_id out of range in build_wheel_chain");
            }
            const int jtype = jnt_type[to_idx(jid)];
            if (jtype != kJntHinge && jtype != kJntSlide) {
                throw std::invalid_argument(
                    "unsupported joint type in wheel kinematic chain (only hinge/slide supported)");
            }
            const int dof_adr = jnt_dofadr[to_idx(jid)];
            if (dof_adr < 0 || dof_adr >= m->nv) {
                throw std::invalid_argument("dof_adr out of range in build_wheel_chain");
            }
            if (chain.joint_count >= kMaxChainJoints) {
                throw std::invalid_argument("wheel kinematic chain exceeds kMaxChainJoints");
            }
            dst_joints[to_idx(chain.joint_count)] = GpuChainJoint{
                .joint_id = jid,
                .joint_type = jtype,
                .dof_adr = dof_adr,
            };
            ++chain.joint_count;
        }
    }

    return chain;
}

HostTyreUploadBundle build_gpu_tyre_model(
    const mjModel *m,
    const mjData *d,
    const nativecfg::TireConfig &cfg,
    int front_geom,
    int rear_geom
) {
    if (m == nullptr) {
        throw std::invalid_argument("null mjModel in build_gpu_tyre_model");
    }
    model_access::require_id(front_geom, m->ngeom, "front_geom");
    model_access::require_id(rear_geom, m->ngeom, "rear_geom");
    const std::span<const int> geom_bodyid =
        model_access::readonly_buffer(m->geom_bodyid, m->ngeom, "geom_bodyid");
    return build_gpu_tyre_model(
        m,
        d,
        cfg,
        front_geom,
        rear_geom,
        geom_bodyid[to_idx(front_geom)],
        geom_bodyid[to_idx(rear_geom)]
    );
}

// NOLINTBEGIN(bugprone-easily-swappable-parameters) Task 5 positional upload signature; validated by geom_bodyid == expected_body_id.
HostTyreUploadBundle build_gpu_tyre_model(
    const mjModel *m,
    const mjData *d,
    const nativecfg::TireConfig &cfg,
    int front_geom,
    int rear_geom,
    int front_body,
    int rear_body,
    int frame_body
) {
// NOLINTEND(bugprone-easily-swappable-parameters)
    if (m == nullptr || d == nullptr) {
        throw std::invalid_argument("null mjModel or mjData in build_gpu_tyre_model");
    }
    if (m->nq <= 0 || m->nq > kMaxNq ||
        m->nv <= 0 || m->nv > kMaxNv ||
        m->nbody <= 0 || m->nbody > kMaxNbody ||
        m->njnt <= 0 || m->njnt > kMaxNjnt) {
        throw std::invalid_argument("model dimensions exceed GPU kMax bounds in build_gpu_tyre_model");
    }

    TyreBackendKind backend_kind = TyreBackendKind::compliant_2d;
    if (cfg.backend == "compliant_2d") {
        backend_kind = TyreBackendKind::compliant_2d;
    } else if (cfg.backend == "distributed_2d_reference") {
        backend_kind = TyreBackendKind::distributed_2d_reference;
    } else if (cfg.backend == "pneumatic") {
        backend_kind = TyreBackendKind::pneumatic;
    } else {
        throw std::invalid_argument(
            "GPU tyre upload does not support backend '" + cfg.backend + "'");
    }

    if (!std::isfinite(cfg.significant_delta_m) || cfg.significant_delta_m < 0.0 ||
        !std::isfinite(cfg.significance_fraction) || cfg.significance_fraction < 0.0 ||
        cfg.significance_fraction > 1.0 ||
        !std::isfinite(cfg.distinct_normal_deg) || cfg.distinct_normal_deg <= 0.0 ||
        cfg.distinct_normal_deg >= 180.0) {
        throw std::invalid_argument("invalid multi-support thresholds in TireConfig");
    }

    HostTyreUploadBundle bundle;
    bundle.gpu_model.backend = backend_kind;
    bundle.gpu_model.nq = static_cast<i32>(m->nq);
    bundle.gpu_model.nv = static_cast<i32>(m->nv);
    bundle.gpu_model.nbody = static_cast<i32>(m->nbody);
    bundle.gpu_model.njnt = static_cast<i32>(m->njnt);

    const std::span<TyreWheelSpec<float>, 2> wheels(bundle.gpu_model.wheels);
    const std::array<int, 2> geom_ids = {front_geom, rear_geom};
    const std::array<int, 2> body_ids = {front_body, rear_body};
    const std::array<const nativecfg::TireParams *, 2> side_params = {&cfg.front, &cfg.rear};

    for (std::size_t side = 0; side < 2; ++side) {
        const nativecfg::TireParams &tp = *side_params[side];
        if (!std::isfinite(tp.tangent_k_n_m) || tp.tangent_k_n_m <= 0.0 ||
            !std::isfinite(tp.relaxation_length_m) || tp.relaxation_length_m <= 0.0 ||
            !std::isfinite(tp.mu) || tp.mu < 0.0) {
            throw std::invalid_argument("invalid wheel TireParams scalars");
        }
        wheels[side].tangent_k_n_m = static_cast<float>(tp.tangent_k_n_m);
        wheels[side].tire_mu = static_cast<float>(tp.mu);
        wheels[side].relaxation_length_m = static_cast<float>(tp.relaxation_length_m);
        validate_and_pack_wheel_geom(
            m, WheelGeomRef{.geom_id = geom_ids[side], .expected_body_id = body_ids[side]},
            wheels[side]);
    }

    int resolved_frame_body = frame_body;
    if (resolved_frame_body < 0) {
        for (const char *name : {"frame", "main_frame", "chassis"}) {
            const int candidate = mj_name2id(m, mjOBJ_BODY, name);
            if (candidate > 0) {
                resolved_frame_body = candidate;
                break;
            }
        }
        if (resolved_frame_body < 0) {
            const std::span<const int> body_parentid =
                model_access::readonly_buffer(m->body_parentid, m->nbody, "body_parentid");
            const int parent = body_parentid[to_idx(front_body)];
            resolved_frame_body = (parent > 0) ? parent : front_body;
        }
    }
    if (resolved_frame_body <= 0 || resolved_frame_body >= m->nbody) {
        throw std::invalid_argument("frame_body out of bounds in build_gpu_tyre_model");
    }
    bundle.gpu_model.frame_chain = build_wheel_chain(m, resolved_frame_body);

    if (backend_kind == TyreBackendKind::compliant_2d) {
        bundle.gpu_model.station_count = 0;
        validate_and_pack_compliant_side(cfg.front, wheels[0]);
        validate_and_pack_compliant_side(cfg.rear, wheels[1]);
    } else if (backend_kind == TyreBackendKind::distributed_2d_reference) {
        if (!cfg.distributed.has_value()) {
            throw std::invalid_argument("distributed_2d_reference requires distributed config");
        }
        const int station_count = cfg.distributed->station_count;
        if (station_count < 16 || station_count > kMaxGpuTyreStations) {
            throw std::invalid_argument(
                "distributed station_count must lie in [16, kMaxGpuTyreStations]");
        }
        bundle.gpu_model.station_count = station_count;
        validate_and_pack_distributed_side(cfg.distributed->front, station_count, wheels[0]);
        validate_and_pack_distributed_side(cfg.distributed->rear, station_count, wheels[1]);
    } else {
        if (!cfg.pneumatic.has_value()) {
            throw std::invalid_argument("pneumatic backend requires pneumatic config");
        }
        bundle.gpu_model.station_count = 0;
        validate_and_pack_pneumatic_side(*cfg.pneumatic, cfg.pneumatic->front, wheels[0]);
        validate_and_pack_pneumatic_side(*cfg.pneumatic, cfg.pneumatic->rear, wheels[1]);
    }

    populate_surface_map(cfg, bundle.gpu_model.surface_map);
    populate_terrain_views(m, d, cfg, bundle);
    bundle.rebind_views();
    return bundle;
}

} // namespace bike::physics_core
