#include "config_types.hpp"
#include "contact/tabulated_radial.hpp"
#include "model_access.hpp"
#include "physics_core/common.hpp"
#include "physics_core/cpu_exact_toolkit.hpp"
#include "physics_core/gpu_toolkit.hpp"
#include "physics_core/tyre_compliant.hpp"
#include "physics_core/tyre_pneumatic_brush.hpp"
#include "physics_core/tyre_pneumatic_carcass.hpp"
#include "physics_core/tyre_pneumatic_geometry.hpp"
#include "physics_core/tyre_pneumatic_model.hpp"
#include "physics_core/tyre_tabulated.hpp"
#include "physics_core/tyre_types.hpp"
#include "pneumatic/brush.hpp"
#include "pneumatic/carcass.hpp"
#include "pneumatic/geometry.hpp"
#include "pneumatic/model.hpp"
#include "pneumatic/profile.hpp"
#include "writers/tire_common.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

using bike::physics_core::CompliantWheelParams;
using bike::physics_core::CompliantWheelStatePod;
using bike::physics_core::CpuExactToolkit;
using bike::physics_core::ErrCode;
using bike::physics_core::GpuChainJoint;
using bike::physics_core::GpuPneumaticStepScratch;
using bike::physics_core::GpuPneumaticWheelOutput;
using bike::physics_core::GpuPneumaticWheelState;
using bike::physics_core::GpuToolkit;
using bike::physics_core::kJntHinge;
using bike::physics_core::kJntSlide;
using bike::physics_core::kMaxPneumaticPatches;
using bike::physics_core::kMaxPneumaticRays;
using bike::physics_core::kMaxRadialTableKnots;
using bike::physics_core::PneumaticBrushResult;
using bike::physics_core::PneumaticCarcassScratch;
using bike::physics_core::PneumaticCarcassStatePod;
using bike::physics_core::PneumaticRayPatch;
using bike::physics_core::PneumaticRayRing;
using bike::physics_core::PneumaticSurfaceMap;
using bike::physics_core::PneumaticSurfaceSample;
using bike::physics_core::PneumaticTyreSpecPod;
using bike::physics_core::PneumaticWheelSpec;
using bike::physics_core::PolylineView;
using bike::physics_core::RadialEvalOutput;
using bike::physics_core::Status;
using bike::physics_core::TabulatedRadialIo;
using bike::physics_core::TabulatedRadialParams;
using bike::physics_core::TyreBackendKind;
using bike::physics_core::TyreModel;
using bike::physics_core::TyreVec3;
using bike::physics_core::i32;

void require(bool cond, std::string_view msg) {
    if (!cond) {
        throw std::runtime_error(std::string(msg));
    }
}

void require_bitwise_eq(double a, double b, std::string_view msg) {
    if (std::bit_cast<std::uint64_t>(a) != std::bit_cast<std::uint64_t>(b)) {
        throw std::runtime_error(
            std::string(msg) + ": bitwise mismatch (" + std::to_string(a) +
            " vs " + std::to_string(b) + ")");
    }
}

void require_near(double actual, double expected, double tol, std::string_view msg) {
    if (!std::isfinite(actual) || !std::isfinite(expected)) {
        throw std::runtime_error(std::string(msg) + ": non-finite value");
    }
    const double scale = std::max({1.0, std::abs(actual), std::abs(expected)});
    const double diff = std::abs(actual - expected);
    if (diff > tol * scale) {
        throw std::runtime_error(
            std::string(msg) + ": diff=" + std::to_string(diff) +
            " tol*scale=" + std::to_string(tol * scale));
    }
}

nativecfg::PneumaticTyreSpec make_front_tyre_cfg() {
    return nativecfg::PneumaticTyreSpec{
        .name = "Schwalbe Magic Mary 29x2.8",
        .outer_radius_mm = 379.0,
        .rim_radius_mm = 320.0,
        .section_height_mm = 68.0,
        .pressure_bar = 1.2,
        .crr_reference = 0.011,
        .casing_width_mm = 65.7,
        .compressed_casing_mm = 6.0,
        .area_factor = 0.509,
        .carcass_stiffness_n_mm2 = 0.136,
        .contact_length_factor = 0.851,
        .rate_stiffening = 0.22,
        .rate_relaxation_s = 0.2,
        .loss_factor = 0.055,
        .hysteresis_rate_eps_mps = 0.01,
        .relaxation_length_mm = 60.0,
        .rim_stiffness_n_mm2 = 28.0,
        .tread_loss_crr = 0.00586
    };
}

template <class Real>
PneumaticTyreSpecPod<Real> make_front_tyre_pod() {
    const auto c = make_front_tyre_cfg();
    return {
        .outer_radius_mm = static_cast<Real>(c.outer_radius_mm),
        .rim_radius_mm = static_cast<Real>(c.rim_radius_mm),
        .section_height_mm = static_cast<Real>(c.section_height_mm),
        .pressure_bar = static_cast<Real>(c.pressure_bar),
        .crr_reference = static_cast<Real>(c.crr_reference),
        .casing_width_mm = static_cast<Real>(c.casing_width_mm),
        .compressed_casing_mm = static_cast<Real>(c.compressed_casing_mm),
        .area_factor = static_cast<Real>(c.area_factor),
        .carcass_stiffness_n_mm2 = static_cast<Real>(c.carcass_stiffness_n_mm2),
        .contact_length_factor = static_cast<Real>(c.contact_length_factor),
        .rate_stiffening = static_cast<Real>(c.rate_stiffening),
        .rate_relaxation_s = static_cast<Real>(c.rate_relaxation_s),
        .loss_factor = static_cast<Real>(c.loss_factor),
        .hysteresis_rate_eps_mps = static_cast<Real>(c.hysteresis_rate_eps_mps),
        .relaxation_length_mm = static_cast<Real>(c.relaxation_length_mm),
        .rim_stiffness_n_mm2 = static_cast<Real>(c.rim_stiffness_n_mm2),
        .tread_loss_crr = static_cast<Real>(c.tread_loss_crr)
    };
}

nativecfg::SurfaceSpec make_asphalt_cfg() {
    return {
        .name = "asphalt",
        .mu_peak = 1.05,
        .mu_slide = 0.75,
        .slip_stiffness_per_load = 15.0,
        .stribeck_speed_mps = 4.5,
        .crr_delta = 0.0,
        .load_sensitivity = 0.06
    };
}

nativecfg::SurfaceSpec make_hardpack_cfg() {
    return {
        .name = "hardpack",
        .mu_peak = 0.80,
        .mu_slide = 0.60,
        .slip_stiffness_per_load = 12.0,
        .stribeck_speed_mps = 4.5,
        .crr_delta = 0.0025,
        .load_sensitivity = 0.08
    };
}

nativecfg::SurfaceSpec make_scree_cfg() {
    return {
        .name = "scree",
        .mu_peak = 0.45,
        .mu_slide = 0.38,
        .slip_stiffness_per_load = 4.5,
        .stribeck_speed_mps = 4.5,
        .crr_delta = 0.0080,
        .load_sensitivity = 0.15
    };
}

template <class Real>
PneumaticSurfaceSample<Real> to_pod_surface(const nativecfg::SurfaceSpec &s) {
    return {
        .mu_peak = static_cast<Real>(s.mu_peak),
        .mu_slide = static_cast<Real>(s.mu_slide),
        .slip_stiffness_per_load = static_cast<Real>(s.slip_stiffness_per_load),
        .stribeck_speed_mps = static_cast<Real>(s.stribeck_speed_mps),
        .crr_delta = static_cast<Real>(s.crr_delta),
        .load_sensitivity = static_cast<Real>(s.load_sensitivity)
    };
}

nativecfg::SurfaceMap make_mixed_surface_map_cfg(double blend_m) {
    nativecfg::SurfaceMap map{};
    map.surface = make_asphalt_cfg();
    map.sections.push_back({
        .start_m = 0.5,
        .end_m = 1.5,
        .surface = make_hardpack_cfg(),
        .blend_m = blend_m
    });
    map.sections.push_back({
        .start_m = 1.5,
        .end_m = 3.0,
        .surface = make_scree_cfg(),
        .blend_m = blend_m
    });
    return map;
}

template <class Real>
PneumaticSurfaceMap<Real> make_mixed_surface_map_pod(Real blend_m) {
    const auto cfg = make_mixed_surface_map_cfg(static_cast<double>(blend_m));
    PneumaticSurfaceMap<Real> map{};
    map.has_surface_map = 1;
    map.default_surface = to_pod_surface<Real>(cfg.surface);
    map.default_surface_id = 0;
    map.section_count = static_cast<i32>(cfg.sections.size());
    const std::span map_secs{map.sections};
    for (std::size_t i = 0; i < cfg.sections.size(); ++i) {
        map_secs[i].start_m = static_cast<Real>(cfg.sections[i].start_m);
        map_secs[i].end_m = static_cast<Real>(cfg.sections[i].end_m);
        map_secs[i].surface = to_pod_surface<Real>(cfg.sections[i].surface);
        map_secs[i].blend_m = static_cast<Real>(cfg.sections[i].blend_m);
        map_secs[i].surface_id = static_cast<i32>(i + 1);
    }
    return map;
}

std::vector<double> make_bumpy_z(double x0, double dx, std::size_t n) {
    std::vector<double> z(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        const double x = x0 + dx * static_cast<double>(i);
        // Smooth bump + trough so patch count changes and rim strike can be probed.
        const double bump = 0.018 * std::exp(-std::pow((x - 0.85) / 0.07, 2.0));
        const double step = 0.014 * std::sin(8.0 * x);
        z[i] = bump + step;
    }
    return z;
}

void test_tabulated_radial() {
    const std::vector<double> defl = {0.0, 0.005, 0.015, 0.030, 0.055};
    const std::vector<double> forc = {0.0, 420.0, 1550.0, 3900.0, 9200.0};
    const nativecfg::TireTable tbl_cfg{.deflection_m = defl, .force_n = forc};
    const contactlaw::TabulatedRadial cpu_ref(tbl_cfg);

    auto p64_ptr = std::make_unique<TabulatedRadialParams<double>>();
    auto p32_ptr = std::make_unique<TabulatedRadialParams<float>>();
    auto &p64 = *p64_ptr;
    auto &p32 = *p32_ptr;
    ErrCode err = ErrCode::none;
    require(bike::physics_core::init_tabulated_radial_params<double, CpuExactToolkit>(
                static_cast<i32>(defl.size()), defl.data(), forc.data(), p64, err) ==
                Status::ok,
            "init_tabulated_radial_params fp64");

    std::vector<float> defl_f(defl.size()), forc_f(forc.size());
    for (std::size_t i = 0; i < defl.size(); ++i) {
        defl_f[i] = static_cast<float>(defl[i]);
        forc_f[i] = static_cast<float>(forc[i]);
    }
    require(bike::physics_core::init_tabulated_radial_params<float, GpuToolkit>(
                static_cast<i32>(defl_f.size()), defl_f.data(), forc_f.data(), p32, err) ==
                Status::ok,
            "init_tabulated_radial_params fp32");

    // Validation error checks
    auto bad_ptr = std::make_unique<TabulatedRadialParams<double>>();
    auto &bad = *bad_ptr;
    const double bad_x[2] = {0.001, 0.01};
    const double bad_f[2] = {0.0, 100.0};
    require(bike::physics_core::init_tabulated_radial_params<double, CpuExactToolkit>(
                2, bad_x, bad_f, bad, err) == Status::invalid_argument,
            "tabulated reject nonzero origin");
    require(bike::physics_core::init_tabulated_radial_params<double, CpuExactToolkit>(
                kMaxRadialTableKnots + 1, defl.data(), forc.data(), bad, err) ==
                Status::out_of_bounds,
            "tabulated reject > kMaxRadialTableKnots");

    // Out-of-bounds domain check
    double out_f = 0.0, out_e = 0.0;
    require(bike::physics_core::tabulated_radial_elastic<double, CpuExactToolkit>(
                &p64, 0.060, out_f, out_e, err) == Status::out_of_bounds &&
                err == ErrCode::outside_domain,
            "tabulated out_of_bounds check");

    // Nonpositive check
    require(bike::physics_core::tabulated_radial_contact<double, CpuExactToolkit>(
                &p64, -0.002, 0.1, 0.08, 0.01, out_f, out_e, err) == Status::ok &&
                out_f == 0.0 && out_e == 0.0,
            "tabulated nonpositive penetration");

    const std::array<double, 7> probes = {
        0.0, 0.0025, 0.005, 0.010, 0.015, 0.042, 0.055};
    for (const double d : probes) {
        const auto [ref_f, ref_e] = cpu_ref.elastic_response(d);
        require(bike::physics_core::tabulated_radial_elastic<double, CpuExactToolkit>(
                    &p64, d, out_f, out_e, err) == Status::ok,
                "tabulated elastic fp64");
        require_bitwise_eq(out_f, ref_f, "tabulated elastic force bitwise");
        require_bitwise_eq(out_e, ref_e, "tabulated elastic energy bitwise");

        float f32 = 0.0f, e32 = 0.0f;
        require(bike::physics_core::tabulated_radial_elastic<float, GpuToolkit>(
                    &p32, static_cast<float>(d), f32, e32, err) == Status::ok,
                "tabulated elastic fp32");
        require_near(static_cast<double>(f32), ref_f, 1e-5, "tabulated elastic f32");
        require_near(static_cast<double>(e32), ref_e, 1e-5, "tabulated elastic e32");

        const auto [ref_cf, ref_ce] = cpu_ref.normal_contact(d, -0.04, 0.08, 0.015);
        require(bike::physics_core::tabulated_radial_contact<double, CpuExactToolkit>(
                    &p64, d, -0.04, 0.08, 0.015, out_f, out_e, err) == Status::ok,
                "tabulated contact fp64");
        require_bitwise_eq(out_f, ref_cf, "tabulated contact force bitwise");
        require_bitwise_eq(out_e, ref_ce, "tabulated contact energy bitwise");

        require(bike::physics_core::tabulated_radial_contact<float, GpuToolkit>(
                    &p32, static_cast<float>(d), -0.04f, 0.08f, 0.015f, f32, e32, err) ==
                    Status::ok,
                "tabulated contact fp32");
        require_near(static_cast<double>(f32), ref_cf, 1e-5, "tabulated contact f32");
    }

    // CompliantWheelParams + TabulatedRadialIo check
    auto cwp_ptr = std::make_unique<CompliantWheelParams<double>>();
    auto &cwp = *cwp_ptr;
    cwp.loss_factor = 0.08;
    cwp.hysteresis_rate_eps_mps = 0.015;
    cwp.radial_table = p64;
    cwp.use_tabulated = 1;
    RadialEvalOutput<double> rad_out{};
    require(bike::physics_core::eval_compliant_radial<double, CpuExactToolkit>(
                &cwp, 0.022, 0.03, rad_out, err) == Status::ok,
            "eval_compliant_radial tabulated");
    const auto [exp_cf, exp_ce] = cpu_ref.normal_contact(0.022, 0.03, 0.08, 0.015);
    require_bitwise_eq(rad_out.normal, exp_cf, "eval_compliant_radial normal");
    require_bitwise_eq(rad_out.energy, exp_ce, "eval_compliant_radial energy");

    std::vector<TabulatedRadialParams<double>> tables{p64, p64};
    TabulatedRadialIo<double, CpuExactToolkit, const TabulatedRadialParams<double> *> io{
        tables.data(), {1, 1}};
    auto m64 = std::make_unique<TyreModel<double>>();
    m64->backend = TyreBackendKind::compliant_2d;
    m64->nq = 3;
    m64->nv = 3;
    m64->nbody = 2;
    m64->njnt = 3;
    m64->distinct_normal_cosine = 0.866;
    m64->wheels[0].radius_m = 0.36;
    m64->wheels[0].tangent_k_n_m = 85000.0;
    m64->wheels[0].tire_mu = 0.95;
    m64->wheels[0].relaxation_length_m = 0.045;
    m64->wheels[0].compliant.loss_factor = 0.08;
    m64->wheels[0].compliant.hysteresis_rate_eps_mps = 0.015;
    m64->wheels[0].compliant.has_tabulated_radial = 1;
    const std::array<double, 3> center = {0.5, 0.0, 0.34};
    const std::array<double, 3> qvel = {2.0, -0.02, 5.5};
    std::array<double, 3> qfrc = {0.0, 0.0, 0.0};
    CompliantWheelStatePod<double> prev_st{}, next_st{};
    auto terrain_fn = [](double cx, double cz, double r, bool, i32,
                         bike::physics_core::PolylineContactResult<double> &out,
                         ErrCode &e) noexcept -> Status {
        std::span{out.point}[0] = cx;
        std::span{out.point}[1] = 0.0;
        std::span{out.normal}[0] = 0.0;
        std::span{out.normal}[1] = 1.0;
        out.delta = r - cz;
        out.segment_id = 0;
        e = ErrCode::none;
        return Status::ok;
    };
    auto jac_fn = [](i32, const bike::physics_core::CompliantJacInput<double> &in,
                     const double *qv_ptr, bike::physics_core::CompliantJacOutput<double> &out,
                     ErrCode &e) noexcept -> Status {
        const auto qv = model_access::readonly_buffer(qv_ptr, 3, "qv");
        out.velocity = {qv[0] + (in.point.z - in.center.z) * qv[2], 0.0,
                        qv[1] - (in.point.x - in.center.x) * qv[2]};
        out.center_velocity = {qv[0], 0.0, qv[1]};
        e = ErrCode::none;
        return Status::ok;
    };
    auto accum_fn = [](i32, const TyreVec3<double> &, const TyreVec3<double> &fw,
                       double *qf_ptr, ErrCode &e) noexcept -> Status {
        const auto qf = model_access::mutable_buffer(qf_ptr, 3, "qf");
        qf[0] += fw.x;
        qf[2] += fw.z;
        e = ErrCode::none;
        return Status::ok;
    };
    require(bike::physics_core::step_compliant_wheel<double, CpuExactToolkit>(
                m64.get(), 0, 0.00125, center.data(), qvel.data(), &prev_st, &next_st, qfrc.data(),
                terrain_fn, jac_fn, accum_fn, io, err) == Status::ok,
            "step_compliant_wheel with TabulatedRadialIo");
    require(qfrc[2] > 0.0, "step_compliant_wheel tabulated produced positive normal force");
    std::cout << "[test_tyre_pneumatic_core] TabulatedRadial PASS\n";
}

void test_pneumatic_geometry_and_carcass() {
    const auto tyre_cfg = make_front_tyre_cfg();
    const auto tyre64 = make_front_tyre_pod<double>();
    const auto tyre32 = make_front_tyre_pod<float>();
    const double half_rad = 75.0 * (std::numbers::pi / 180.0);

    const biketyre::RayRing cpu_ring64(64, half_rad);
    const biketyre::RayRing cpu_ring256(256, half_rad);

    auto ring64 = std::make_unique<PneumaticRayRing<double>>();
    auto ring256 = std::make_unique<PneumaticRayRing<double>>();
    auto ring32 = std::make_unique<PneumaticRayRing<float>>();
    ErrCode err = ErrCode::none;
    require(bike::physics_core::init_pneumatic_ray_ring<double, CpuExactToolkit>(
                64, half_rad, *ring64, err) == Status::ok,
            "init ring 64 fp64");
    require(bike::physics_core::init_pneumatic_ray_ring<double, CpuExactToolkit>(
                256, half_rad, *ring256, err) == Status::ok,
            "init ring 256 fp64");
    require(bike::physics_core::init_pneumatic_ray_ring<float, GpuToolkit>(
                64, static_cast<float>(half_rad), *ring32, err) == Status::ok,
            "init ring 64 fp32");

    for (std::size_t i = 0; i < 64; ++i) {
        require_bitwise_eq(std::span{ring64->theta_rad}[i], cpu_ring64.theta_rad()[i], "ring64 theta");
        require_bitwise_eq(std::span{ring64->ux}[i], cpu_ring64.ux()[i], "ring64 ux");
        require_bitwise_eq(std::span{ring64->uz}[i], cpu_ring64.uz()[i], "ring64 uz");
        require_near(static_cast<double>(std::span{ring32->ux}[i]), cpu_ring64.ux()[i], 1e-6, "ring32 ux");
    }

    // Road profile & intersect parity (bumpy road + airborne)
    const double x0 = 0.0, dx = 0.01;
    const auto z = make_bumpy_z(x0, dx, 400);
    const biketyre::RoadProfile cpu_road(x0, dx, z);
    std::vector<float> px32(z.size()), pz32(z.size());
    for (std::size_t i = 0; i < z.size(); ++i) {
        px32[i] = static_cast<float>(cpu_road.x_m()[i]);
        pz32[i] = static_cast<float>(z[i]);
    }

    const auto hits_cpu = biketyre::intersect(cpu_ring256, 0.85, 0.355, 0.379, cpu_road);
    auto scr64 = std::make_unique<GpuPneumaticStepScratch<double>>();
    auto scr32 = std::make_unique<GpuPneumaticStepScratch<float>>();
    bike::physics_core::u8 cov = 0, air = 0;
    require(bike::physics_core::pneumatic_intersect_uniform_grid<double, CpuExactToolkit>(
                ring256.get(), 0.85, 0.355, 0.379, x0, dx,
                static_cast<i32>(z.size()),
                cpu_road.x_m().data(), cpu_road.z_m().data(),
                scr64->r_m, scr64->delta_m, scr64->road_x_m, scr64->road_z_m,
                cov, air, err) == Status::ok,
            "pneumatic_intersect_uniform_grid 256");
    require((cov != 0) == hits_cpu.coverage_event && (air != 0) == hits_cpu.airborne,
            "intersect flags match");
    for (std::size_t i = 0; i < 256; ++i) {
        require_bitwise_eq(std::span{scr64->delta_m}[i], hits_cpu.delta_m[i], "intersect256 delta");
        require_bitwise_eq(std::span{scr64->road_x_m}[i], hits_cpu.road_x_m[i], "intersect256 road_x");
        require_bitwise_eq(std::span{scr64->road_z_m}[i], hits_cpu.road_z_m[i], "intersect256 road_z");
    }

    // Also verify PolylineView intersection matches uniform grid
    PolylineView<double> pl64{};
    pl64.px = cpu_road.x_m().data();
    pl64.pz = cpu_road.z_m().data();
    pl64.num_vertices = static_cast<i32>(z.size());
    std::vector<double> pl_r(kMaxPneumaticRays), pl_d(kMaxPneumaticRays),
        pl_rx(kMaxPneumaticRays), pl_rz(kMaxPneumaticRays);
    require(bike::physics_core::pneumatic_intersect_polyline<double, CpuExactToolkit>(
                ring256.get(), 0.85, 0.355, 0.379, pl64,
                pl_r.data(), pl_d.data(), pl_rx.data(), pl_rz.data(),
                cov, air, err) == Status::ok,
            "pneumatic_intersect_polyline 256");
    for (std::size_t i = 0; i < 256; ++i) {
        require_near(pl_d[i], hits_cpu.delta_m[i], 1e-13, "polyline intersect delta");
    }

    // Patches check
    const auto cpu_p = biketyre::patches(hits_cpu.delta_m);
    std::array<PneumaticRayPatch, kMaxPneumaticPatches> core_p{};
    i32 core_pc = 0;
    require(bike::physics_core::pneumatic_patches<double>(
                256, scr64->delta_m, kMaxPneumaticPatches, core_p.data(), core_pc, err) ==
                Status::ok,
            "pneumatic_patches");
    require(static_cast<std::size_t>(core_pc) == cpu_p.size(), "patch count match");
    for (std::size_t k = 0; k < cpu_p.size(); ++k) {
        require(core_p[k].start == cpu_p[k].first && core_p[k].stop == cpu_p[k].second,
                "patch bounds match");
    }

    // Stage A chord integral Taylor & zero-stiffness Maxwell potential
    const double rho = 0.5 * tyre_cfg.casing_width_mm * 0.001;
    const double ds = 0.379 * ring256->dtheta_rad;
    for (const double d : {0.0, 1e-18, 1e-12, 1e-6, 0.005, rho, 1.5 * rho}) {
        const double w_ref = biketyre::casing_chord_integral_m2(d, rho);
        const double w_core =
            bike::physics_core::casing_chord_integral_m2<double, CpuExactToolkit>(d, rho);
        require_bitwise_eq(w_core, w_ref, "casing_chord_integral_m2 bitwise");
        require(w_core >= 0.0, "casing_chord_integral_m2 nonnegative");

        const double w_f32 = static_cast<double>(
            bike::physics_core::casing_chord_integral_m2<float, GpuToolkit>(
                static_cast<float>(d), static_cast<float>(rho)));
        require(w_f32 >= 0.0, "casing_chord_integral_m2 fp32 nonnegative");
        if (d > 1e-6) {
            require_near(w_f32, w_ref, 1e-4, "casing_chord_integral_m2 fp32");
        }
    }

    // Zero carcass stiffness guard in ray_maxwell_energy_j
    auto zero_kc_cfg = tyre_cfg;
    zero_kc_cfg.carcass_stiffness_n_mm2 = 0.0;
    auto zero_kc_pod = tyre64;
    zero_kc_pod.carcass_stiffness_n_mm2 = 0.0;
    const double max_zero_kc = bike::physics_core::ray_maxwell_energy_j<double, CpuExactToolkit>(
        0.04, 100.0, 5.0, zero_kc_pod, ds);
    require_bitwise_eq(
        max_zero_kc,
        biketyre::ray_maxwell_energy_j(0.04, 100.0, 5.0, zero_kc_cfg, ds),
        "ray_maxwell_energy_j zero kc");

    // Multi-step Carcass parity including rim strike and release
    biketyre::CarcassState cpu_cs(64);
    auto core_cs64 = std::make_unique<PneumaticCarcassStatePod<double>>();
    auto core_cs32 = std::make_unique<PneumaticCarcassStatePod<float>>();
    core_cs64->n_rays = 64;
    core_cs32->n_rays = 64;
    auto cscr64 = std::make_unique<PneumaticCarcassScratch<double>>();
    auto cscr32 = std::make_unique<PneumaticCarcassScratch<float>>();

    // Rim strike deflection = (379 - 320 - 6) = 53 mm = 0.053 m.
    const std::array<double, 5> hub_zs = {0.355, 0.340, 0.315, 0.345, 0.370};
    bool saw_rim_active = false;
    bool saw_rim_completed = false;
    for (int step = 0; step < 5; ++step) {
        const double hx = 0.80 + 0.02 * static_cast<double>(step);
        const double hz = hub_zs[static_cast<std::size_t>(step)];
        const auto h = biketyre::intersect(cpu_ring64, hx, hz, 0.379, cpu_road);
        const auto cres_cpu = biketyre::evaluate_carcass(
            cpu_ring64, h, tyre_cfg, cpu_cs, 0.0005, 8.0, 3.0);

        require(bike::physics_core::pneumatic_evaluate_carcass<double, CpuExactToolkit>(
                    ring64.get(), h.delta_m.data(), h.road_x_m.data(),
                    h.road_z_m.data(), tyre64, core_cs64.get(), core_cs64.get(),
                    0.0005, 8.0, 3.0, cscr64.get(), err) == Status::ok,
                "carcass step fp64");

        std::vector<float> d32(64), rx32(64), rz32(64);
        for (std::size_t i = 0; i < 64; ++i) {
            d32[i] = static_cast<float>(h.delta_m[i]);
            rx32[i] = static_cast<float>(h.road_x_m[i]);
            rz32[i] = static_cast<float>(h.road_z_m[i]);
        }
        require(bike::physics_core::pneumatic_evaluate_carcass<float, GpuToolkit>(
                    ring32.get(), d32.data(), rx32.data(), rz32.data(),
                    tyre32, core_cs32.get(), core_cs32.get(),
                    0.0005f, 8.0f, 3.0f, cscr32.get(), err) == Status::ok,
                "carcass step fp32");

        require_near(cscr64->force_x_n, cres_cpu.force_x_n, 1e-12, "carcass force_x fp64");
        require_near(cscr64->force_z_n, cres_cpu.force_z_n, 1e-12, "carcass force_z fp64");
        require_near(cscr64->rim_energy_j, cres_cpu.rim_energy_j, 1e-12, "carcass rim_e fp64");
        require_near(cscr64->dissipated_power_w, cres_cpu.dissipated_power_w, 1e-12,
                     "carcass power fp64");

        require_near(static_cast<double>(cscr32->force_z_n), cres_cpu.force_z_n, 1e-4,
                     "carcass force_z fp32");
        require_near(static_cast<double>(cscr32->dissipated_power_w),
                     cres_cpu.dissipated_power_w, 1e-3,
                     "carcass power fp32");

        if (cscr64->rim_strike_active != 0) {
            saw_rim_active = true;
        }
        if (cscr64->rim_event.valid != 0) {
            saw_rim_completed = true;
            require(cres_cpu.rim_event.has_value(), "cpu rim_event should also complete");
            require_near(cscr64->rim_event.peak_rim_force_n,
                         cres_cpu.rim_event->peak_rim_force_n, 1e-12,
                         "rim event peak_rim_force_n");
        }
    }
    require(saw_rim_active && saw_rim_completed, "rim strike FSM exercised");
    std::cout << "[test_tyre_pneumatic_core] Pneumatic Geometry + Carcass PASS\n";
}

void test_pneumatic_brush_and_stage_a() {
    const auto map_cfg = make_mixed_surface_map_cfg(0.4);
    const auto map64 = make_mixed_surface_map_pod<double>(0.4);
    const auto map32 = make_mixed_surface_map_pod<float>(0.4f);

    // Surface evaluation & C1 blend parity
    ErrCode err = ErrCode::none;
    for (const double x : {0.1, 0.35, 0.5, 0.65, 1.0, 1.35, 1.5, 1.65, 2.5}) {
        const auto s_ref = tirecommon::eval_surface(map_cfg, x);
        PneumaticSurfaceSample<double> s64{};
        PneumaticSurfaceSample<float> s32{};
        i32 sid64 = -1, sid32 = -1;
        require(bike::physics_core::eval_surface_sample<double, CpuExactToolkit>(
                    &map64, x, s64, sid64, err) == Status::ok,
                "eval_surface_sample fp64");
        require(bike::physics_core::eval_surface_sample<float, GpuToolkit>(
                    &map32, static_cast<float>(x), s32, sid32, err) == Status::ok,
                "eval_surface_sample fp32");
        require_bitwise_eq(s64.mu_peak, s_ref.mu_peak, "surface mu_peak bitwise");
        require_bitwise_eq(s64.crr_delta, s_ref.crr_delta, "surface crr_delta bitwise");
        require_bitwise_eq(s64.load_sensitivity, s_ref.load_sensitivity,
                           "surface load_sensitivity bitwise");
        require_near(static_cast<double>(s32.mu_peak), s_ref.mu_peak, 1e-6, "surface fp32");
    }

    // Load sensitivity overflow-safe guard in FP32
    const float lsf_huge = bike::physics_core::load_sensitivity_factor<float, GpuToolkit>(
        1e25f, 0.15f);
    require(std::isfinite(lsf_huge) && lsf_huge > 0.0f && lsf_huge < 1e-10f,
            "FP32 load_sensitivity_factor huge load overflow-safe");

    // Lumped brush bitwise parity
    const auto asp_cfg = make_asphalt_cfg();
    const auto asp64 = to_pod_surface<double>(asp_cfg);
    for (const double kp : {0.0, 0.04, -0.15, -1.0, std::numeric_limits<double>::infinity()}) {
        const auto ref = biketyre::lumped_brush(kp, 450.0, 0.045, asp_cfg, std::nullopt, 0.8);
        PneumaticBrushResult<double> core_res{};
        require(bike::physics_core::pneumatic_lumped_brush<double, CpuExactToolkit>(
                    kp, 450.0, 0.045, asp64, false, 0.0, 0.8, core_res, err) == Status::ok,
                "pneumatic_lumped_brush fp64");
        require_bitwise_eq(core_res.force_n, ref.force_n, "lumped_brush force bitwise");
        require_bitwise_eq(core_res.friction_coefficient, ref.friction_coefficient,
                           "lumped_brush mu bitwise");
        require((core_res.fully_sliding != 0) == ref.fully_sliding, "lumped_brush sliding");
    }

    // Discretised brush heterogeneous parity
    constexpr i32 kN = 16;
    std::vector<double> loads64(kN, 28.0);
    std::vector<float> loads32(kN, 28.0f);
    std::vector<tirecommon::SurfaceSample> cpu_samples(kN);
    std::vector<PneumaticSurfaceSample<double>> pod_samples64(kN);
    std::vector<PneumaticSurfaceSample<float>> pod_samples32(kN);
    for (i32 i = 0; i < kN; ++i) {
        const double x = 1.32 + 0.025 * static_cast<double>(i);
        cpu_samples[static_cast<std::size_t>(i)] = tirecommon::eval_surface(map_cfg, x);
        i32 sid = 0;
        (void)bike::physics_core::eval_surface_sample<double, CpuExactToolkit>(
            &map64, x, pod_samples64[static_cast<std::size_t>(i)], sid, err);
        (void)bike::physics_core::eval_surface_sample<float, GpuToolkit>(
            &map32, static_cast<float>(x),
            pod_samples32[static_cast<std::size_t>(i)], sid, err);
    }

    biketyre::DiscretisedBrush cpu_brush(kN);
    double kp64 = 0.0;
    float kp32 = 0.0f;
    std::array<double, kN> br64{}, upd64{}, kbuf64{}, mbuf64{};
    std::array<float, kN> br32{}, upd32{}, kbuf32{}, mbuf32{};
    for (int step = 0; step < 5; ++step) {
        const auto ref = cpu_brush.step_heterogeneous(
            loads64, 0.04, 4.0, -0.35, 0.06, cpu_samples, 448.0, 720.0, 0.00025);
        PneumaticBrushResult<double> res64{};
        PneumaticBrushResult<float> res32{};
        require(bike::physics_core::discretised_brush_step_heterogeneous<double, CpuExactToolkit>(
                    kN, loads64.data(), 0.04, 4.0, -0.35, 0.06,
                    pod_samples64.data(), 448.0, 720.0, 0.00025,
                    kp64, br64.data(), upd64.data(), kbuf64.data(),
                    mbuf64.data(), res64, err) == Status::ok,
                "discretised_brush_step_heterogeneous fp64");
        require(bike::physics_core::discretised_brush_step_heterogeneous<float, GpuToolkit>(
                    kN, loads32.data(), 0.04f, 4.0f, -0.35f, 0.06f,
                    pod_samples32.data(), 448.0f, 720.0f, 0.00025f,
                    kp32, br32.data(), upd32.data(), kbuf32.data(),
                    mbuf32.data(), res32, err) == Status::ok,
                "discretised_brush_step_heterogeneous fp32");
        require_near(res64.force_n, ref.force_n, 1e-12, "hetero brush force fp64");
        require_near(res64.friction_coefficient, ref.friction_coefficient, 1e-12,
                     "hetero brush mu fp64");
        require_near(static_cast<double>(res32.force_n), ref.force_n, 1e-4,
                     "hetero brush force fp32");
    }
    std::cout << "[test_tyre_pneumatic_core] Pneumatic Brush + Stage A PASS\n";
}

void test_whole_wheel_model_parity(bool discretised, i32 n_rays, double dt_s) {
    const auto tyre_cfg = make_front_tyre_cfg();
    const double x0 = 0.0, dx = 0.01;
    const auto z = make_bumpy_z(x0, dx, 400);
    const biketyre::RoadProfile cpu_road(x0, dx, z);
    const auto map_cfg = make_mixed_surface_map_cfg(0.4);

    biketyre::PneumaticTyreModel cpu_model(tyre_cfg, n_rays, 75.0, discretised);
    const auto map64 = make_mixed_surface_map_pod<double>(0.4);
    const auto map32 = make_mixed_surface_map_pod<float>(0.4f);

    auto spec64 = std::make_unique<PneumaticWheelSpec<double>>();
    auto spec32 = std::make_unique<PneumaticWheelSpec<float>>();
    ErrCode err = ErrCode::none;
    spec64->tyre = make_front_tyre_pod<double>();
    spec64->discretised_brush = discretised ? 1 : 0;
    require(bike::physics_core::init_pneumatic_ray_ring<double, CpuExactToolkit>(
                n_rays, 75.0 * (std::numbers::pi / 180.0), spec64->ring, err) ==
                Status::ok,
            "whole wheel init ring64");

    spec32->tyre = make_front_tyre_pod<float>();
    spec32->discretised_brush = discretised ? 1 : 0;
    require(bike::physics_core::init_pneumatic_ray_ring<float, GpuToolkit>(
                n_rays, static_cast<float>(75.0 * (std::numbers::pi / 180.0)),
                spec32->ring, err) == Status::ok,
            "whole wheel init ring32");

    std::vector<float> px32(z.size()), pz32(z.size());
    for (std::size_t i = 0; i < z.size(); ++i) {
        px32[i] = static_cast<float>(cpu_road.x_m()[i]);
        pz32[i] = static_cast<float>(z[i]);
    }

    auto st64 = std::make_unique<GpuPneumaticWheelState<double>>();
    auto st32 = std::make_unique<GpuPneumaticWheelState<float>>();
    bike::physics_core::init_pneumatic_wheel_state<double>(n_rays, st64.get());
    bike::physics_core::init_pneumatic_wheel_state<float>(n_rays, st32.get());
    auto scr64 = std::make_unique<GpuPneumaticStepScratch<double>>();
    auto scr32 = std::make_unique<GpuPneumaticStepScratch<float>>();

    // Roll from asphalt (x=0.35) -> hardpack transition (0.5) -> bump (0.85) ->
    // scree transition (1.5) -> airborne jump (z=0.50) -> decay.
    for (int step = 0; step < 30; ++step) {
        const double hx = 0.35 + 0.045 * static_cast<double>(step);
        double hz = 0.352;
        if (step >= 10 && step <= 13) {
            hz = 0.335; // deep compression over bump/transition
        } else if (step >= 26) {
            hz = 0.55;  // airborne Maxwell decay
        }
        const std::array<double, 3> hub64 = {hx, 0.0, hz};
        const std::array<double, 3> vel64 = {4.2, 0.0, (step % 2 == 0) ? -0.04 : 0.03};
        const double omega64 = 10.6;

        const auto &cpu_out = cpu_model.evaluate(
            hub64, vel64, omega64, cpu_road, map_cfg, dt_s);

        GpuPneumaticWheelOutput<double> out64{};
        require(bike::physics_core::evaluate_pneumatic_wheel<double, CpuExactToolkit>(
                    spec64.get(), &map64, hub64.data(), vel64.data(), omega64,
                    cpu_road.x_m().data(), cpu_road.z_m().data(),
                    static_cast<i32>(z.size()), x0, dx, true, dt_s,
                    st64.get(), st64.get(), out64, scr64.get(), err) == Status::ok,
                "evaluate_pneumatic_wheel fp64");

        const std::array<float, 3> hub32 = {
            static_cast<float>(hx), 0.0f, static_cast<float>(hz)};
        const std::array<float, 3> vel32 = {
            static_cast<float>(vel64[0]), 0.0f, static_cast<float>(vel64[2])};
        GpuPneumaticWheelOutput<float> out32{};
        require(bike::physics_core::evaluate_pneumatic_wheel<float, GpuToolkit>(
                    spec32.get(), &map32, hub32.data(), vel32.data(),
                    static_cast<float>(omega64),
                    px32.data(), pz32.data(),
                    static_cast<i32>(z.size()), static_cast<float>(x0),
                    static_cast<float>(dx), true, static_cast<float>(dt_s),
                    st32.get(), st32.get(), out32, scr32.get(), err) == Status::ok,
                "evaluate_pneumatic_wheel fp32");

        require_near(std::span{out64.force_world_n}[0], cpu_out.force_world_n[0], 1e-12,
                     "whole wheel fx fp64");
        require_near(std::span{out64.force_world_n}[2], cpu_out.force_world_n[2], 1e-12,
                     "whole wheel fz fp64");
        require_near(out64.normal_load_n, cpu_out.normal_load_n, 1e-12,
                     "whole wheel normal_load fp64");
        require_near(out64.rolling_crr, cpu_out.rolling_crr, 1e-12,
                     "whole wheel rolling_crr fp64");
        require_near(out64.carcass_dissipated_power_w, cpu_out.carcass_dissipated_power_w,
                     1e-12, "whole wheel carcass power fp64");
        require_near(out64.dissipated_power_w, cpu_out.dissipated_power_w, 1e-12,
                     "whole wheel total power fp64");
        require(out64.airborne == (cpu_out.airborne ? 1 : 0), "airborne match");

        double cpu_shear = 0.0;
        for (const auto &p : cpu_out.patches()) {
            cpu_shear += p.shear_energy_j;
        }
        require_near(out64.shear_energy_j, cpu_shear, 1e-12,
                     "whole wheel shear_energy_j fp64");

        // FP32 parity against CPU FP64
        require_near(static_cast<double>(std::span{out32.force_world_n}[0]),
                     cpu_out.force_world_n[0], 1e-3, "whole wheel fx fp32");
        require_near(static_cast<double>(std::span{out32.force_world_n}[2]),
                     cpu_out.force_world_n[2], 1e-4, "whole wheel fz fp32");
        require_near(static_cast<double>(out32.rolling_crr),
                     cpu_out.rolling_crr, 1e-4, "whole wheel crr fp32");

        // Stored energy consistency check
        double stored64 = 0.0;
        require(bike::physics_core::pneumatic_stored_energy<double, CpuExactToolkit>(
                    spec64.get(), cpu_road.x_m().data(), cpu_road.z_m().data(),
                    static_cast<i32>(z.size()), x0, dx, true, hx, hz,
                    st64.get(), out64.shear_energy_j, stored64, scr64.get(), err) ==
                    Status::ok,
                "pneumatic_stored_energy fp64");
        require_bitwise_eq(
            stored64,
            out64.radial_energy_j + out64.maxwell_energy_j +
                out64.rim_energy_j + out64.shear_energy_j,
            "stored_energy == sum of components");
    }
    std::cout << "[test_tyre_pneumatic_core] Whole Wheel ("
              << (discretised ? "detailed 256" : "fast 64") << ") Parity PASS\n";
}

void test_step_pneumatic_wheel_wrench_and_energy() {
    // 3-DOF planar wheel chain: [x_slide, z_slide, pitch_hinge] on body 1.
    auto spec64 = std::make_unique<PneumaticWheelSpec<double>>();
    ErrCode err = ErrCode::none;
    spec64->tyre = make_front_tyre_pod<double>();
    spec64->discretised_brush = 1;
    spec64->wheel_body_id = 1;
    spec64->nv = 3;
    spec64->njnt = 3;
    spec64->chain.wheel_body_id = 1;
    spec64->chain.njnt = 3;
    spec64->chain.joint_count = 3;
    const std::span chain_joints{spec64->chain.joints};
    chain_joints[0] = GpuChainJoint{.joint_id = 0, .joint_type = kJntSlide, .dof_adr = 0};
    chain_joints[1] = GpuChainJoint{.joint_id = 1, .joint_type = kJntSlide, .dof_adr = 1};
    chain_joints[2] = GpuChainJoint{.joint_id = 2, .joint_type = kJntHinge, .dof_adr = 2};
    require(bike::physics_core::init_pneumatic_ray_ring<double, CpuExactToolkit>(
                256, 75.0 * (std::numbers::pi / 180.0), spec64->ring, err) ==
                Status::ok,
            "wrench test ring");

    const auto map64 = make_mixed_surface_map_pod<double>(0.4);
    const double x0 = 0.0, dx = 0.01;
    const std::vector<double> z(200, 0.0);
    std::vector<double> px(200, 0.0);
    for (std::size_t i = 0; i < 200; ++i) {
        px[i] = x0 + dx * static_cast<double>(i);
    }

    auto st64 = std::make_unique<GpuPneumaticWheelState<double>>();
    bike::physics_core::init_pneumatic_wheel_state<double>(256, st64.get());
    auto scr64 = std::make_unique<GpuPneumaticStepScratch<double>>();

    const std::array<double, 6> xpos = {0.0, 0.0, 0.0, 0.55, 0.0, 0.355};
    const std::array<double, 18> xmat = {
        1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
        1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0
    };
    const std::array<double, 9> xanchor = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.55, 0.0, 0.355};
    const std::array<double, 9> xaxis = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0};
    const std::array<double, 3> qvel = {4.0, -0.05, 10.5};
    std::array<double, 3> qfrc = {0.0, 0.0, 0.0};
    GpuPneumaticWheelOutput<double> out{};

    require(bike::physics_core::step_pneumatic_wheel<double, CpuExactToolkit>(
                spec64.get(), &map64, 0.00025,
                px.data(), z.data(), 200, x0, dx, true,
                xpos.data(), xmat.data(), xanchor.data(), xaxis.data(), qvel.data(),
                st64.get(), st64.get(), qfrc.data(), out, scr64.get(), err) == Status::ok,
            "step_pneumatic_wheel flat");

    require(out.normal_load_n > 0.0, "positive normal load");
    require(out.effective_radius_m > 0.30 && out.effective_radius_m < 0.38,
            "effective radius reasonable");
    require(out.rolling_crr > spec64->tyre.tread_loss_crr,
            "hardpack/blend rolling_crr includes crr_delta");
    require(out.rolling_moment_nm < 0.0, "rolling moment opposes positive omega");
    require(out.dissipated_power_w >= out.carcass_dissipated_power_w,
            "total dissipation >= carcass dissipation");

    // Verify qfrc matches centroid force + axial rolling moment:
    // qfrc[0] = F_world_x, qfrc[1] = F_world_z,
    // qfrc[2] = (centroid - hub) x F_world |_y + M_roll
    require_near(qfrc[0], std::span{out.force_world_n}[0], 1e-12, "qfrc[0] == fx");
    require_near(qfrc[1], std::span{out.force_world_n}[2], 1e-12, "qfrc[1] == fz");
    const double expected_tau =
        ((std::span{scr64->patch_centroids}[2] - xpos[5]) * std::span{scr64->patch_forces}[0] -
         (std::span{scr64->patch_centroids}[0] - xpos[3]) * std::span{scr64->patch_forces}[2]) +
        out.rolling_moment_nm;
    require_near(qfrc[2], expected_tau, 1e-12, "qfrc[2] == patch moment + M_roll");

    std::cout << "[test_tyre_pneumatic_core] step_pneumatic_wheel Wrench + Energy PASS\n";
}

void test_surface_null_uniform_and_polyline_contracts() {
    static_assert(sizeof(GpuPneumaticStepScratch<float>) <= 24576,
                  "GpuPneumaticStepScratch<float> must stay compact (< 24 KB)");

    ErrCode err = ErrCode::none;

    // 1) Null surface_map rejection in brush + model + step entrypoints
    PneumaticSurfaceSample<double> s_null{};
    i32 sid_null = -1;
    require(bike::physics_core::eval_surface_sample<double, CpuExactToolkit>(
                static_cast<const PneumaticSurfaceMap<double> *>(nullptr),
                0.5, s_null, sid_null, err) == Status::invalid_argument,
            "eval_surface_sample rejects null map");
    i32 sec_null = 0;
    bool is_uni_null = true;
    require(bike::physics_core::pneumatic_uniform_surface_for_span<double, CpuExactToolkit>(
                static_cast<const PneumaticSurfaceMap<double> *>(nullptr),
                0.0, 1.0, sec_null, is_uni_null, err) == Status::invalid_argument,
            "pneumatic_uniform_surface_for_span rejects null map");

    auto spec64 = std::make_unique<PneumaticWheelSpec<double>>();
    spec64->tyre = make_front_tyre_pod<double>();
    spec64->discretised_brush = 1;
    spec64->nv = 3;
    spec64->njnt = 3;
    spec64->wheel_body_id = 1;
    spec64->chain.wheel_body_id = 1;
    spec64->chain.njnt = 3;
    spec64->chain.joint_count = 3;
    const std::span chain_joints{spec64->chain.joints};
    chain_joints[0] = GpuChainJoint{.joint_id = 0, .joint_type = kJntSlide, .dof_adr = 0};
    chain_joints[1] = GpuChainJoint{.joint_id = 1, .joint_type = kJntSlide, .dof_adr = 1};
    chain_joints[2] = GpuChainJoint{.joint_id = 2, .joint_type = kJntHinge, .dof_adr = 2};
    require(bike::physics_core::init_pneumatic_ray_ring<double, CpuExactToolkit>(
                64, 75.0 * (std::numbers::pi / 180.0), spec64->ring, err) ==
                Status::ok,
            "contract ring64");

    std::vector<double> px(100), pz(100, 0.0);
    for (std::size_t i = 0; i < px.size(); ++i) {
        px[i] = 0.02 * static_cast<double>(i);
    }
    auto st_in = std::make_unique<GpuPneumaticWheelState<double>>();
    auto st_out = std::make_unique<GpuPneumaticWheelState<double>>();
    bike::physics_core::init_pneumatic_wheel_state<double>(64, st_in.get());
    auto scr64 = std::make_unique<GpuPneumaticStepScratch<double>>();
    const std::array<double, 3> hub_air{0.55, 0.0, 0.80};
    const std::array<double, 3> vel_zero{0.0, 0.0, 0.0};
    GpuPneumaticWheelOutput<double> out{};
    require(bike::physics_core::evaluate_pneumatic_wheel<double, CpuExactToolkit>(
                spec64.get(),
                static_cast<const PneumaticSurfaceMap<double> *>(nullptr),
                hub_air.data(), vel_zero.data(), 0.0,
                px.data(), pz.data(), 100, 0.0, 0.02, true, 0.0005,
                st_in.get(), st_out.get(), out, scr64.get(), err) ==
                Status::invalid_argument,
            "evaluate_pneumatic_wheel rejects null surface_map");

    const std::array<double, 6> xpos = {0.0, 0.0, 0.0, 0.55, 0.0, 0.355};
    const std::array<double, 18> xmat = {
        1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
        1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0
    };
    const std::array<double, 9> xanchor = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.55, 0.0, 0.355};
    const std::array<double, 9> xaxis = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0};
    const std::array<double, 3> qvel = {4.0, -0.05, 10.5};
    std::array<double, 3> qfrc = {0.0, 0.0, 0.0};
    require(bike::physics_core::step_pneumatic_wheel<double, CpuExactToolkit>(
                spec64.get(),
                static_cast<const PneumaticSurfaceMap<double> *>(nullptr),
                0.0005, px.data(), pz.data(), 100, 0.0, 0.02, true,
                xpos.data(), xmat.data(), xanchor.data(), xaxis.data(),
                qvel.data(), st_in.get(), st_out.get(), qfrc.data(),
                out, scr64.get(), err) == Status::invalid_argument,
            "step_pneumatic_wheel rejects null surface_map");

    // 2) Uniform surface mode (has_surface_map == 0) returns default_surface
    PneumaticSurfaceMap<double> uni_map{};
    uni_map.has_surface_map = 0;
    uni_map.section_count = 0;
    uni_map.default_surface_id = 7;
    uni_map.default_surface = to_pod_surface<double>(make_asphalt_cfg());
    PneumaticSurfaceSample<double> s_uni{};
    i32 sid_uni = -1;
    require(bike::physics_core::eval_surface_sample<double, CpuExactToolkit>(
                &uni_map, 0.5, s_uni, sid_uni, err) == Status::ok,
            "eval_surface_sample uniform ok");
    require(sid_uni == 7, "eval_surface_sample uniform returns default_surface_id");
    require_bitwise_eq(s_uni.mu_peak, uni_map.default_surface.mu_peak,
                       "eval_surface_sample uniform returns default mu_peak");
    require(bike::physics_core::evaluate_pneumatic_wheel<double, CpuExactToolkit>(
                spec64.get(), &uni_map, hub_air.data(), vel_zero.data(), 0.0,
                px.data(), pz.data(), 100, 0.0, 0.02, true, 0.0005,
                st_in.get(), st_out.get(), out, scr64.get(), err) == Status::ok,
            "evaluate_pneumatic_wheel airborne uniform ok");
    require(out.airborne == 1, "airborne flag set");
    require_bitwise_eq(out.friction_coefficient, uni_map.default_surface.mu_peak,
                       "airborne uniform wheel uses default_surface mu_peak");
    require(out.dominant_surface_id == 7,
            "airborne uniform wheel uses default_surface_id");

    // 3) Polyline road (road_is_uniform_grid = false) matches uniform grid
    const std::array<double, 3> hub_loaded{0.55, 0.0, 0.355};
    const std::array<double, 3> vel_loaded{4.0, 0.0, -0.05};
    auto st_grid = std::make_unique<GpuPneumaticWheelState<double>>();
    auto st_poly = std::make_unique<GpuPneumaticWheelState<double>>();
    bike::physics_core::init_pneumatic_wheel_state<double>(64, st_grid.get());
    bike::physics_core::init_pneumatic_wheel_state<double>(64, st_poly.get());
    GpuPneumaticWheelOutput<double> out_grid{}, out_poly{};
    require(bike::physics_core::evaluate_pneumatic_wheel<double, CpuExactToolkit>(
                spec64.get(), &uni_map, hub_loaded.data(), vel_loaded.data(), 10.5,
                px.data(), pz.data(), 100, 0.0, 0.02, true, 0.0005,
                st_in.get(), st_grid.get(), out_grid, scr64.get(), err) == Status::ok,
            "uniform grid eval ok");
    require(bike::physics_core::evaluate_pneumatic_wheel<double, CpuExactToolkit>(
                spec64.get(), &uni_map, hub_loaded.data(), vel_loaded.data(), 10.5,
                px.data(), pz.data(), 100, 0.0, 0.02, false, 0.0005,
                st_in.get(), st_poly.get(), out_poly, scr64.get(), err) == Status::ok,
            "polyline eval ok");
    require_near(out_poly.normal_load_n, out_grid.normal_load_n, 1e-12,
                 "polyline vs uniform grid normal_load");
    require_near(std::span{out_poly.force_world_n}[0],
                 std::span{out_grid.force_world_n}[0], 1e-12,
                 "polyline vs uniform grid fx");

    std::cout << "[test_tyre_pneumatic_core] Null/Uniform/Polyline Contracts PASS\n";
}

} // namespace

int main() {
    test_tabulated_radial();
    test_pneumatic_geometry_and_carcass();
    test_pneumatic_brush_and_stage_a();
    test_whole_wheel_model_parity(false, 64, 0.0005);
    test_whole_wheel_model_parity(true, 256, 0.00025);
    test_step_pneumatic_wheel_wrench_and_energy();
    test_surface_null_uniform_and_polyline_contracts();
    std::cout << "ALL test_tyre_pneumatic_core CHECKS PASSED\n";
    return 0;
}
