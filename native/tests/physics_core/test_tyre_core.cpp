#include "physics_core/common.hpp"
#include "physics_core/cpu_exact_toolkit.hpp"
#include "physics_core/gpu_toolkit.hpp"
#include "physics_core/tyre_compliant.hpp"
#include "physics_core/tyre_contact.hpp"
#include "physics_core/tyre_distributed.hpp"
#include "physics_core/tyre_types.hpp"

#if defined(__APPLE__)
#include "test_tyre_core_metal_helper.hpp"
#endif

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

using bike::physics_core::CompliantJacInput;
using bike::physics_core::CompliantJacOutput;
using bike::physics_core::CompliantWheelStatePod;
using bike::physics_core::CpuExactToolkit;
using bike::physics_core::ErrCode;
using bike::physics_core::GpuDistributedWorkspace;
using bike::physics_core::GpuToolkit;
using bike::physics_core::NoOpIo;
using bike::physics_core::PointDistanceResult;
using bike::physics_core::PolylineContactResult;
using bike::physics_core::StationStatePod;
using bike::physics_core::Status;
using bike::physics_core::SurfaceMapView;
using bike::physics_core::TyreBackendKind;
using bike::physics_core::TyreModel;
using bike::physics_core::TyreVec3;
using bike::physics_core::TyreXZ;
using bike::physics_core::i32;

void require(bool cond, std::string_view msg) {
    if (!cond) {
        throw std::runtime_error(std::string(msg));
    }
}

struct RelTarget {
    double expected = 0.0;
    double tol = 0.0;
};

void require_rel(double actual, RelTarget target, std::string_view msg) {
    if (!std::isfinite(actual) || !std::isfinite(target.expected)) {
        throw std::runtime_error(std::string(msg) + ": non-finite value");
    }
    const double scale =
        std::max({1.0, std::abs(actual), std::abs(target.expected)});
    const double diff = std::abs(actual - target.expected);
    if (diff > target.tol * scale) {
        throw std::runtime_error(
            std::string(msg) + ": diff=" + std::to_string(diff) +
            " scale=" + std::to_string(scale));
    }
}

template <class Real>
SurfaceMapView<Real> make_test_surface_map() {
    SurfaceMapView<Real> map{};
    map.has_surface_map = 1;
    map.section_count = 2;
    map.default_surface = {
        .mu_peak = static_cast<Real>(0.90),
        .mu_slide = static_cast<Real>(0.72),
        .slip_stiffness_per_load = static_cast<Real>(12.0),
        .stribeck_speed_mps = static_cast<Real>(0.8),
        .surface_id = 0
    };
    map.sections[0].start_m = static_cast<Real>(0.0);
    map.sections[0].end_m = static_cast<Real>(1.2);
    map.sections[0].surface = {
        .mu_peak = static_cast<Real>(1.05),
        .mu_slide = static_cast<Real>(0.85),
        .slip_stiffness_per_load = static_cast<Real>(15.0),
        .stribeck_speed_mps = static_cast<Real>(0.6),
        .surface_id = 1
    };
    map.sections[1].start_m = static_cast<Real>(1.2);
    map.sections[1].end_m = static_cast<Real>(2.5);
    map.sections[1].surface = {
        .mu_peak = static_cast<Real>(0.65),
        .mu_slide = static_cast<Real>(0.45),
        .slip_stiffness_per_load = static_cast<Real>(10.0),
        .stribeck_speed_mps = static_cast<Real>(0.5),
        .surface_id = 2
    };
    return map;
}

template <class Real>
std::unique_ptr<TyreModel<Real>> make_compliant_model() {
    auto m = std::make_unique<TyreModel<Real>>();
    m->backend = TyreBackendKind::compliant_2d;
    m->nq = 3;
    m->nv = 3;
    m->nbody = 2;
    m->njnt = 3;
    m->distinct_normal_cosine =
        static_cast<Real>(std::cos(30.0 * std::numbers::pi / 180.0));
    m->wheels[0].geom_id = 0;
    m->wheels[0].radius_m = static_cast<Real>(0.36);
    m->wheels[0].tangent_k_n_m = static_cast<Real>(85000.0);
    m->wheels[0].tire_mu = static_cast<Real>(0.95);
    m->wheels[0].relaxation_length_m = static_cast<Real>(0.045);
    m->wheels[0].compliant.radial_k_n_m = static_cast<Real>(145000.0);
    m->wheels[0].compliant.loss_factor = static_cast<Real>(0.08);
    m->wheels[0].compliant.hysteresis_rate_eps_mps = static_cast<Real>(0.015);
    m->wheels[0].compliant.valid_load_min_n = static_cast<Real>(0.0);
    m->wheels[0].compliant.valid_load_max_n = static_cast<Real>(5000.0);
    m->wheels[0].compliant.has_tabulated_radial = 0;
    m->surface_map = make_test_surface_map<Real>();
    return m;
}

template <class Real, class TK>
struct TestJacFn {
    std::array<Real, 9> jac{};
    std::array<Real, 9> jac_c{};

    Status operator()(
        i32 /*side*/,
        const CompliantJacInput<Real> &in_j,
        const Real *qvel,
        CompliantJacOutput<Real> &out_j,
        ErrCode &err
    ) noexcept {
        jac.fill(static_cast<Real>(0));
        jac_c.fill(static_cast<Real>(0));
        // 3-DOF planar wheel: q = [x, z, pitch]
        jac[0 * 3 + 0] = static_cast<Real>(1);
        jac[0 * 3 + 2] = in_j.point.z - in_j.center.z;
        jac[2 * 3 + 1] = static_cast<Real>(1);
        jac[2 * 3 + 2] = -(in_j.point.x - in_j.center.x);
        jac_c[0 * 3 + 0] = static_cast<Real>(1);
        jac_c[2 * 3 + 1] = static_cast<Real>(1);
        std::array<Real, 3> v_arr{};
        std::array<Real, 3> cv_arr{};
        TK::gemv_n3(3, jac.data(), qvel, v_arr.data());
        TK::gemv_n3(3, jac_c.data(), qvel, cv_arr.data());
        out_j.velocity = {.x = v_arr[0], .y = v_arr[1], .z = v_arr[2]};
        out_j.center_velocity = {.x = cv_arr[0], .y = cv_arr[1], .z = cv_arr[2]};
        err = ErrCode::none;
        return Status::ok;
    }
};

template <class Real, class TK>
struct TestAccumFn {
    const std::array<Real, 9> *jac = nullptr;
    std::array<Real, 3> *qfrc_ref = nullptr;

    Status operator()(
        i32 /*side*/,
        const TyreVec3<Real> & /*p*/,
        const TyreVec3<Real> &force_world,
        Real * /*out_qfrc*/,
        ErrCode &err
    ) const noexcept {
        const std::array<Real, 3> fw = {
            force_world.x, force_world.y, force_world.z};
        std::array<Real, 3> tmp{};
        TK::gemv_t3(3, jac->data(), fw.data(), tmp.data());
        for (std::size_t i = 0; i < 3; ++i) {
            (*qfrc_ref)[i] += tmp[i];
        }
        err = ErrCode::none;
        return Status::ok;
    }
};

void test_tyre_contact_primitives() {
    const auto map64 = make_test_surface_map<double>();
    const auto map32 = make_test_surface_map<float>();

    double mu64 = 0.0;
    float mu32 = 0.0f;
    i32 sec64 = -1;
    i32 sec32 = -1;
    ErrCode err64 = ErrCode::none;
    ErrCode err32 = ErrCode::none;

    require(bike::physics_core::effective_friction<double, CpuExactToolkit>(
                0.95, &map64, 0.5, 0.25, mu64, sec64, err64) == Status::ok,
            "effective_friction fp64 section 0");
    require(bike::physics_core::effective_friction<float, GpuToolkit>(
                0.95f, &map32, 0.5f, 0.25f, mu32, sec32, err32) == Status::ok,
            "effective_friction fp32 section 0");
    require(sec64 == 0 && sec32 == 0, "effective_friction section index 0");
    require_rel(mu64, {.expected = static_cast<double>(mu32), .tol = 1e-6},
                "effective_friction mu");

    // Normal hysteresis + passivity check
    double fn64 = 0.0;
    double en64 = 0.0;
    float fn32 = 0.0f;
    float en32 = 0.0f;
    require(bike::physics_core::normal_contact_hysteresis<double, CpuExactToolkit>(
                0.012, -0.05, 145000.0, 0.08, 0.015, fn64, en64, err64) == Status::ok,
            "normal_contact_hysteresis fp64");
    require(bike::physics_core::normal_contact_hysteresis<float, GpuToolkit>(
                0.012f, -0.05f, 145000.0f, 0.08f, 0.015f, fn32, en32, err32) == Status::ok,
            "normal_contact_hysteresis fp32");
    require_rel(fn64, {.expected = static_cast<double>(fn32), .tol = 1e-6},
                "normal_contact_hysteresis fn");
    require_rel(en64, {.expected = static_cast<double>(en32), .tol = 1e-6},
                "normal_contact_hysteresis energy");

    // Brush step check
    double xi64 = 0.0;
    double fx64 = 0.0;
    double loss64 = 0.0;
    float xi32 = 0.0f;
    float fx32 = 0.0f;
    float loss32 = 0.0f;
    require(bike::physics_core::brush_step<double, CpuExactToolkit>(
                0.002, 0.15, 3.0, fn64, 85000.0, mu64, 0.045, 0.00125,
                xi64, fx64, loss64, err64) == Status::ok,
            "brush_step fp64");
    require(bike::physics_core::brush_step<float, GpuToolkit>(
                0.002f, 0.15f, 3.0f, fn32, 85000.0f, mu32, 0.045f, 0.00125f,
                xi32, fx32, loss32, err32) == Status::ok,
            "brush_step fp32");
    require_rel(xi64, {.expected = static_cast<double>(xi32), .tol = 1e-6},
                "brush_step xi");
    require_rel(fx64, {.expected = static_cast<double>(fx32), .tol = 1e-6},
                "brush_step force");
    require_rel(loss64, {.expected = static_cast<double>(loss32), .tol = 1e-5},
                "brush_step loss");

    // Overflow check (spring_force = 1e20 * 1e300 = +Inf)
    require(bike::physics_core::normal_contact_hysteresis<double, CpuExactToolkit>(
                1e20, 0.1, 1e300, 0.08, 0.015, fn64, en64, err64) == Status::overflow,
            "normal_contact_hysteresis overflow on huge k*delta");
}

template <class Real, class TK>
auto make_test_terrain_fn() {
    return [](Real cx, Real cz, Real radius, bool /*has_prev*/,
              i32 /*prev_seg*/, PolylineContactResult<Real> &out,
              ErrCode &err) noexcept -> Status {
        if (cx < static_cast<Real>(1.0)) {
            out.point[0] = cx;
            out.point[1] = static_cast<Real>(0.0);
            out.normal[0] = static_cast<Real>(0.0);
            out.normal[1] = static_cast<Real>(1.0);
            out.delta = radius - cz;
            out.segment_id = 0;
        } else if (cx < static_cast<Real>(2.0)) {
            const Real slope = static_cast<Real>(0.15);
            const Real inv_len =
                static_cast<Real>(1.0) /
                TK::sqrt(static_cast<Real>(1.0) + slope * slope);
            const Real nx = -slope * inv_len;
            const Real nz = inv_len;
            const Real h = slope * (cx - static_cast<Real>(1.0));
            const Real dist = (cz - h) * nz;
            out.point[0] = cx - dist * nx;
            out.point[1] = cz - dist * nz;
            out.normal[0] = nx;
            out.normal[1] = nz;
            out.delta = radius - dist;
            out.segment_id = 1;
        } else {
            out.point[0] = cx;
            out.point[1] = static_cast<Real>(0.15);
            out.normal[0] = static_cast<Real>(0.0);
            out.normal[1] = static_cast<Real>(1.0);
            out.delta = radius - (cz - static_cast<Real>(0.15));
            out.segment_id = 4;
        }
        out.multi_support = 0;
        err = ErrCode::none;
        return Status::ok;
    };
}

void test_compliant_step_fp32_vs_fp64() {
    const auto model64 = make_compliant_model<double>();
    const auto model32 = make_compliant_model<float>();

    CompliantWheelStatePod<double> st64{};
    CompliantWheelStatePod<float> st32{};

    auto terr64 = make_test_terrain_fn<double, CpuExactToolkit>();
    auto terr32 = make_test_terrain_fn<float, GpuToolkit>();
    TestJacFn<double, CpuExactToolkit> jac64{};
    TestJacFn<float, GpuToolkit> jac32{};
    NoOpIo io{};

    constexpr double kDt = 0.00125;
    for (int tick = 0; tick < 24; ++tick) {
        const double cx = 0.4 + 0.11 * static_cast<double>(tick);
        double cz = 0.348;
        if (tick >= 8 && tick <= 13) {
            cz = 0.55; // airborne above slope
        } else if (tick >= 14 && tick <= 18) {
            cz = 0.35 + 0.15 * (cx - 1.0);
        } else if (tick >= 19) {
            cz = 0.495;
        }
        const std::array<double, 3> center64 = {cx, 0.0, cz};
        const std::array<float, 3> center32 = {
            static_cast<float>(cx), 0.0f, static_cast<float>(cz)};
        const std::array<double, 3> qvel64 = {2.8, -0.03, 7.6};
        const std::array<float, 3> qvel32 = {2.8f, -0.03f, 7.6f};

        std::array<double, 3> qfrc64{};
        std::array<float, 3> qfrc32{};
        TestAccumFn<double, CpuExactToolkit> accum64{
            .jac = &jac64.jac, .qfrc_ref = &qfrc64};
        TestAccumFn<float, GpuToolkit> accum32{
            .jac = &jac32.jac, .qfrc_ref = &qfrc32};
        CompliantWheelStatePod<double> next64{};
        CompliantWheelStatePod<float> next32{};
        ErrCode err64 = ErrCode::none;
        ErrCode err32 = ErrCode::none;

        const Status s64 =
            bike::physics_core::step_compliant_wheel<double, CpuExactToolkit>(
                model64.get(), 0, kDt, center64.data(), qvel64.data(),
                &st64, &next64, qfrc64.data(),
                terr64, jac64, accum64, io, err64);
        const Status s32 =
            bike::physics_core::step_compliant_wheel<float, GpuToolkit>(
                model32.get(), 0, static_cast<float>(kDt), center32.data(),
                qvel32.data(), &st32, &next32, qfrc32.data(),
                terr32, jac32, accum32, io, err32);

        require(s64 == Status::ok && s32 == Status::ok, "compliant step status ok");
        require(next64.segment == next32.segment, "compliant step segment parity");
        require(next64.has_segment == 1 && next32.has_segment == 1,
                "airborne segment persistence");

        for (std::size_t i = 0; i < 3; ++i) {
            require_rel(
                qfrc64[i],
                {.expected = static_cast<double>(qfrc32[i]), .tol = 5e-5},
                "compliant qfrc fp32 vs fp64 tick " + std::to_string(tick));
        }
        require_rel(
            next64.xi, {.expected = static_cast<double>(next32.xi), .tol = 5e-5},
            "compliant xi fp32 vs fp64 tick " + std::to_string(tick));

        if (tick >= 8 && tick <= 13) {
            require(next64.xi == 0.0 && next32.xi == 0.0f, "airborne zero xi");
            require(qfrc64[0] == 0.0 && qfrc64[2] == 0.0, "airborne zero qfrc");
        }
        st64 = next64;
        st32 = next32;
    }
}

template <class Real>
std::unique_ptr<TyreModel<Real>> make_distributed_model(int station_count) {
    auto m = std::make_unique<TyreModel<Real>>();
    m->backend = TyreBackendKind::distributed_2d_reference;
    m->nq = 3;
    m->nv = 3;
    m->nbody = 2;
    m->njnt = 3;
    m->station_count = station_count;
    m->distinct_normal_cosine =
        static_cast<Real>(std::cos(30.0 * std::numbers::pi / 180.0));
    const double two_pi = 2.0 * std::numbers::pi;
    const double weight = two_pi / static_cast<double>(station_count);
    auto &w = m->wheels[0];
    w.geom_id = 0;
    w.radius_m = static_cast<Real>(0.36);
    w.tangent_k_n_m = static_cast<Real>(85000.0);
    w.tire_mu = static_cast<Real>(0.95);
    w.relaxation_length_m = static_cast<Real>(0.045);
    w.station_weight = static_cast<Real>(weight);
    w.distributed_density.knot_count = 3;
    w.distributed_density.knots_m[0] = static_cast<Real>(0.0);
    w.distributed_density.knots_m[1] = static_cast<Real>(0.004);
    w.distributed_density.knots_m[2] = static_cast<Real>(0.012);
    w.distributed_density.stiffness_n_m[0] = static_cast<Real>(300000.0);
    w.distributed_density.stiffness_n_m[1] = static_cast<Real>(50000.0);
    w.distributed_density.stiffness_n_m[2] = static_cast<Real>(0.0);
    w.distributed_density.loss_factor = static_cast<Real>(0.05);
    w.distributed_density.hysteresis_rate_eps_mps = static_cast<Real>(0.01);
    const std::span<Real> dst_local(w.local_xyz);
    for (int i = 0; i < station_count; ++i) {
        const double angle =
            two_pi * static_cast<double>(i) / static_cast<double>(station_count);
        const std::size_t base = static_cast<std::size_t>(i) * 3u;
        dst_local[base + 0u] = static_cast<Real>(0.36 * std::cos(angle));
        dst_local[base + 1u] = static_cast<Real>(0.0);
        dst_local[base + 2u] = static_cast<Real>(0.36 * std::sin(angle));
    }
    m->surface_map = make_test_surface_map<Real>();
    return m;
}

template <class Real>
struct TestDistHeightFn {
    Status operator()(
        Real x,
        Real &out_h,
        ErrCode &err
    ) const noexcept {
        if (x < static_cast<Real>(0.0) || x > static_cast<Real>(10.0)) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
        out_h = x < static_cast<Real>(1.0)
            ? static_cast<Real>(0.0)
            : static_cast<Real>(0.10) * (x - static_cast<Real>(1.0));
        err = ErrCode::none;
        return Status::ok;
    }
};

template <class Real>
struct TestCenterJacFn {
    GpuDistributedWorkspace<Real, 128u, 3u> *ws = nullptr;

    Status operator()(
        const TyreVec3<Real> & /*center*/,
        Real * /*jp_center*/,
        Real * /*jr*/,
        ErrCode &err
    ) const noexcept {
        const std::span<Real, 9> jp_s(ws->jac_center);
        const std::span<Real, 9> jr_s(ws->jr);
        std::ranges::fill(jp_s, static_cast<Real>(0));
        std::ranges::fill(jr_s, static_cast<Real>(0));
        // 3-DOF planar wheel: q = [x, z, pitch]
        jp_s[0 * 3 + 0] = static_cast<Real>(1);
        jp_s[2 * 3 + 1] = static_cast<Real>(1);
        jr_s[1 * 3 + 2] = static_cast<Real>(1);
        err = ErrCode::none;
        return Status::ok;
    }
};

template <class Real, class TK>
auto make_test_dist_query_fn() {
    return [](const TyreXZ<Real> &xz, PointDistanceResult<Real> &out,
              ErrCode &err) noexcept -> Status {
        if (xz.x < static_cast<Real>(1.0)) {
            out.distance = xz.z;
            out.normal[0] = static_cast<Real>(0.0);
            out.normal[1] = static_cast<Real>(1.0);
            out.point[0] = xz.x;
            out.point[1] = static_cast<Real>(0.0);
            out.segment = 0;
        } else {
            const Real slope = static_cast<Real>(0.10);
            const Real inv_len =
                static_cast<Real>(1.0) /
                TK::sqrt(static_cast<Real>(1.0) + slope * slope);
            const Real nx = -slope * inv_len;
            const Real nz = inv_len;
            const Real h = slope * (xz.x - static_cast<Real>(1.0));
            const Real d = (xz.z - h) * nz;
            out.distance = d;
            out.normal[0] = nx;
            out.normal[1] = nz;
            out.point[0] = xz.x - d * nx;
            out.point[1] = xz.z - d * nz;
            out.segment = 1;
        }
        out.nonsmooth = 0;
        err = ErrCode::none;
        return Status::ok;
    };
}

void test_distributed_step_fp32_vs_fp64() {
    constexpr int kStations = 128;
    const auto model64 = make_distributed_model<double>(kStations);
    const auto model32 = make_distributed_model<float>(kStations);

    std::vector<StationStatePod<double>> st64(static_cast<std::size_t>(kStations));
    std::vector<StationStatePod<float>> st32(static_cast<std::size_t>(kStations));
    std::vector<StationStatePod<double>> next64(static_cast<std::size_t>(kStations));
    std::vector<StationStatePod<float>> next32(static_cast<std::size_t>(kStations));

    auto ws64 = std::make_unique<GpuDistributedWorkspace<double, 128u, 3u>>();
    auto ws32 = std::make_unique<GpuDistributedWorkspace<float, 128u, 3u>>();

    TestDistHeightFn<double> hfn64{};
    TestDistHeightFn<float> hfn32{};
    auto qfn64 = make_test_dist_query_fn<double, CpuExactToolkit>();
    auto qfn32 = make_test_dist_query_fn<float, GpuToolkit>();
    TestCenterJacFn<double> jfn64{.ws = ws64.get()};
    TestCenterJacFn<float> jfn32{.ws = ws32.get()};
    NoOpIo io{};

    constexpr double kDt = 0.00125;
    for (int tick = 0; tick < 20; ++tick) {
        const double cx = 0.6 + 0.05 * static_cast<double>(tick);
        double cz = 0.342;
        if (tick >= 10 && tick <= 13) {
            cz = 0.48; // airborne
        } else if (tick >= 14) {
            cz = 0.345 + 0.10 * (cx - 1.0);
        }
        const double pitch = 0.15 * static_cast<double>(tick);
        const double c = std::cos(pitch);
        const double s = std::sin(pitch);
        const std::array<double, 9> xmat64 = {c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c};
        const std::array<float, 9> xmat32 = {
            static_cast<float>(c), 0.0f, static_cast<float>(s),
            0.0f, 1.0f, 0.0f,
            static_cast<float>(-s), 0.0f, static_cast<float>(c)};
        const std::array<double, 3> center64 = {cx, 0.0, cz};
        const std::array<float, 3> center32 = {
            static_cast<float>(cx), 0.0f, static_cast<float>(cz)};
        const std::array<double, 3> qvel64 = {2.6, -0.02, 7.2};
        const std::array<float, 3> qvel32 = {2.6f, -0.02f, 7.2f};

        std::array<double, 3> qfrc64{};
        std::array<float, 3> qfrc32{};
        ErrCode err64 = ErrCode::none;
        ErrCode err32 = ErrCode::none;

        const Status s64 =
            bike::physics_core::step_distributed_wheel<double, CpuExactToolkit>(
                model64.get(), 0, kDt, center64.data(), xmat64.data(),
                qvel64.data(), std::span{model64->wheels[0].local_xyz}.data(),
                static_cast<const double *>(nullptr),
                st64.data(), next64.data(), qfrc64.data(),
                *ws64, hfn64, qfn64, jfn64, io, err64);
        const Status s32 =
            bike::physics_core::step_distributed_wheel<float, GpuToolkit>(
                model32.get(), 0, static_cast<float>(kDt), center32.data(),
                xmat32.data(), qvel32.data(),
                std::span{model32->wheels[0].local_xyz}.data(),
                static_cast<const float *>(nullptr),
                st32.data(), next32.data(), qfrc32.data(),
                *ws32, hfn32, qfn32, jfn32, io, err32);

        require(s64 == Status::ok && s32 == Status::ok,
                "distributed step status ok");
        for (std::size_t i = 0; i < static_cast<std::size_t>(kStations); ++i) {
            require(next64[i].active == next32[i].active,
                    "distributed active station bitmask parity tick " +
                        std::to_string(tick));
            if (next64[i].active != 0) {
                require_rel(
                    next64[i].xi,
                    {.expected = static_cast<double>(next32[i].xi), .tol = 5e-5},
                    "distributed station xi parity");
            }
        }
        for (std::size_t k = 0; k < 3; ++k) {
            require_rel(
                qfrc64[k],
                {.expected = static_cast<double>(qfrc32[k]), .tol = 5e-5},
                "distributed qfrc fp32 vs fp64 tick " + std::to_string(tick));
        }
        st64.swap(next64);
        st32.swap(next32);
    }
}

#if defined(__APPLE__)
void test_metal_tyre_compile() {
    const auto res = bike::physics_core::test_tyre_metal::run_metal_tyre_smoke(
        BIKE_PHYSICS_CORE_SRC_DIR, true);
    require(res.device_available, "Metal device available");
    if (!res.error_message.empty()) {
        throw std::runtime_error(res.error_message);
    }
    require(res.compiled && res.executed,
            "Metal tyre compliant + distributed compile + pipeline");
    require(std::isfinite(res.compliant_xi) && std::abs(res.compliant_qfrc0) > 0.0f,
            "Metal compliant smoke kernel produced non-zero finite force");
    require(std::isfinite(res.distributed_xi0) && std::abs(res.distributed_qfrc0) > 0.0f,
            "Metal distributed smoke kernel produced non-zero finite force");
    std::cout << "[test_tyre_core] Metal compliant + distributed dispatch PASS on "
              << res.device_name << " (comp_qfrc0=" << res.compliant_qfrc0
              << ", dist_qfrc0=" << res.distributed_qfrc0 << ")\n";
}
#endif

} // namespace

int main() {
    try {
        test_tyre_contact_primitives();
        test_compliant_step_fp32_vs_fp64();
        test_distributed_step_fp32_vs_fp64();
#if defined(__APPLE__)
        test_metal_tyre_compile();
#endif
        std::cout << "[test_tyre_core] ALL PASS\n";
        return 0;
    } catch (const std::exception &ex) {
        std::cerr << "[test_tyre_core] FAIL: " << ex.what() << '\n';
        return 1;
    }
}
