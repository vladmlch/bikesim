#include "physics_core/brake_hold.hpp"
#include "physics_core/common.hpp"
#include "physics_core/cpu_exact_toolkit.hpp"
#include "physics_core/gpu_math.hpp"
#include "physics_core/gpu_toolkit.hpp"
#include "physics_core/joint_envelope.hpp"
#include "physics_core/kinematics.hpp"
#include "physics_core/ratchet.hpp"
#include "physics_core/resistance.hpp"
#include "physics_core/suspension.hpp"
#include "physics_core/tyre_types.hpp"
#include "physics_core/tyre_upload.hpp"

#include "config_types.hpp"
#include "drivetrain/transmission.hpp"
#include "engine_call.hpp"
#include "model_access.hpp"
#include "rider/spindle_math.hpp"
#include "runtime/static_brake.hpp"
#include "support/fixture_test.hpp"
#include "writers/resistance.hpp"
#include "writers/suspension.hpp"

#if defined(__APPLE__)
#include "test_sp2_force_laws_metal_helper.hpp"
#endif

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
#include <random>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

using bike::physics_core::BrakeHoldParams;
using bike::physics_core::ChainFrameJpFn;
using bike::physics_core::ChainWheelJrFn;
using bike::physics_core::CpuExactToolkit;
using bike::physics_core::ErrCode;
using bike::physics_core::GpuToolkit;
using bike::physics_core::GpuWheelChain;
using bike::physics_core::ResDiagField;
using bike::physics_core::ResistanceParams;
using bike::physics_core::RiderJointEnvelopeSpec;
using bike::physics_core::Status;
using bike::physics_core::SuspDiagField;
using bike::physics_core::SuspensionParams;
using bike::physics_core::SuspensionResult;
using bike::physics_core::WorldRotationsRebaseState;
using bike::physics_core::i32;
using bike::physics_core::kCassetteCogCount;
using bike::physics_core::u8;

using ModelPtr = std::unique_ptr<mjModel, void (*)(mjModel *)>;
using DataPtr = std::unique_ptr<mjData, void (*)(mjData *)>;

void delete_model(mjModel *m) {
    if (m != nullptr) {
        mj_deleteModel(m);
    }
}

void delete_data(mjData *d) {
    if (d != nullptr) {
        mj_deleteData(d);
    }
}

void require(bool cond, std::string_view msg) {
    bt::require(cond, msg);
}

ModelPtr load_xml_model(const std::filesystem::path &xml_path) {
    std::array<char, 1024> err{};
    mjModel *raw = mj_loadXML(xml_path.c_str(), nullptr, err.data(),
                              static_cast<int>(err.size()));
    require(raw != nullptr,
            "mj_loadXML(" + xml_path.string() + "): " + err.data());
    return {raw, delete_model};
}

void require_bits_f64(double actual, double expected, std::string_view what) {
    if (std::bit_cast<std::uint64_t>(actual) !=
        std::bit_cast<std::uint64_t>(expected)) {
        throw std::runtime_error(std::string(what) + ": 64-bit mismatch");
    }
}

void require_bits_f32(float actual, float expected, std::string_view what) {
    if (std::bit_cast<std::uint32_t>(actual) !=
        std::bit_cast<std::uint32_t>(expected)) {
        throw std::runtime_error(std::string(what) + ": 32-bit mismatch");
    }
}

std::uint32_t float_ulp_diff(float a, float b) {
    if (std::isnan(a) || std::isnan(b)) {
        return std::numeric_limits<std::uint32_t>::max();
    }
    if (a == b) {
        return 0u;
    }
    const auto ua = std::bit_cast<std::uint32_t>(a);
    const auto ub = std::bit_cast<std::uint32_t>(b);
    const auto map_ordered = [](std::uint32_t u) -> std::uint32_t {
        return (u & 0x80000000u) != 0u ? (0x80000000u - u) : (u + 0x80000000u);
    };
    const std::uint32_t oa = map_ordered(ua);
    const std::uint32_t ob = map_ordered(ub);
    return oa > ob ? (oa - ob) : (ob - oa);
}

nativecfg::SuspensionConfig make_stock_suspension_cfg() {
    nativecfg::SuspensionConfig cfg{};
    cfg.physics_mode = "physical";
    cfg.fork_joint = "fork_travel";
    cfg.shock_joint = "shock_stroke";
    cfg.air_spring = {
        .specs = {
            .stanchion_inner_diam_mm = 24.0,
            .total_travel_mm = 180.0,
            .pos_chamber_length_mm = 280.0,
            .neg_chamber_length_mm = 55.0,
            .token_volume_cm3 = 8.0,
            .max_tokens = 4,
            .gamma = 1.40,
            .atm_pressure_pa = 101325.0,
        },
        .num_tokens = 2,
        .gauge_pressure_psi = 85.2,
    };
    cfg.fork_damper = {
        .core = {
            .max_hsc = 4, .max_lsc = 14, .max_reb = 17,
            .hsc_clicks = 2, .lsc_clicks = 7, .rebound_clicks = 9,
            .c_lsc_min = 260.0, .c_lsc_max = 1150.0,
            .c_hsc_min = 180.0, .c_hsc_max = 520.0,
            .c_reb_min = 350.0, .c_reb_max = 1550.0,
            .v_knee_comp = 0.18, .v_knee_reb = 0.22,
        },
        .total_travel_mm = 180.0,
        .hbo_start_mm = 160.0,
        .c_hbo_base = 3500.0,
        .legacy_behavior = false,
    };
    cfg.shock_damper = {
        .core = {
            .max_hsc = 4, .max_lsc = 14, .max_reb = 14,
            .hsc_clicks = 2, .lsc_clicks = 7, .rebound_clicks = 7,
            .c_lsc_min = 650.0, .c_lsc_max = 2900.0,
            .c_hsc_min = 450.0, .c_hsc_max = 1450.0,
            .c_reb_min = 900.0, .c_reb_max = 8000.0,
            .v_knee_comp = 0.16, .v_knee_reb = 0.20,
        },
        .total_stroke_mm = 65.0,
        .max_hbo = 4,
        .hbo_clicks = 2,
        .lockout_firm = false,
        .legacy_behavior = false,
        .hbo_start_mm = 52.0,
        .c_hbo_min = 4000.0,
        .c_hbo_max = 18000.0,
        .lockout_preload_n = 480.0,
        .lockout_stiffness = 15000.0,
    };
    cfg.coil = {
        .rate_n_m = 114600.0,
        .preload_mm = 0.0,
        .stroke_mm = 65.0,
        .bumper_length_mm = 10.0,
        .bumper_peak_n = 7000.0,
        .legacy_behavior = false,
    };
    cfg.end_stops = {
        .stiffness_n_m = 250000.0,
        .damping_n_s_m = 2000.0,
    };
    return cfg;
}

template <class Real>
SuspensionParams<Real> to_core_susp(const nativecfg::SuspensionConfig &c) {
    const auto copy_core = [](const nativecfg::DamperCore &in) {
        return bike::physics_core::DamperCoreParams<Real>{
            .max_hsc = in.max_hsc,
            .max_lsc = in.max_lsc,
            .max_reb = in.max_reb,
            .hsc_clicks = in.hsc_clicks,
            .lsc_clicks = in.lsc_clicks,
            .rebound_clicks = in.rebound_clicks,
            .c_lsc_min = static_cast<Real>(in.c_lsc_min),
            .c_lsc_max = static_cast<Real>(in.c_lsc_max),
            .c_hsc_min = static_cast<Real>(in.c_hsc_min),
            .c_hsc_max = static_cast<Real>(in.c_hsc_max),
            .c_reb_min = static_cast<Real>(in.c_reb_min),
            .c_reb_max = static_cast<Real>(in.c_reb_max),
            .v_knee_comp = static_cast<Real>(in.v_knee_comp),
            .v_knee_reb = static_cast<Real>(in.v_knee_reb),
        };
    };
    return SuspensionParams<Real>{
        .air_spring = {
            .stanchion_inner_diam_mm = static_cast<Real>(c.air_spring.specs.stanchion_inner_diam_mm),
            .total_travel_mm = static_cast<Real>(c.air_spring.specs.total_travel_mm),
            .pos_chamber_length_mm = static_cast<Real>(c.air_spring.specs.pos_chamber_length_mm),
            .neg_chamber_length_mm = static_cast<Real>(c.air_spring.specs.neg_chamber_length_mm),
            .token_volume_cm3 = static_cast<Real>(c.air_spring.specs.token_volume_cm3),
            .max_tokens = c.air_spring.specs.max_tokens,
            .gamma = static_cast<Real>(c.air_spring.specs.gamma),
            .atm_pressure_pa = static_cast<Real>(c.air_spring.specs.atm_pressure_pa),
            .num_tokens = c.air_spring.num_tokens,
            .gauge_pressure_psi = static_cast<Real>(c.air_spring.gauge_pressure_psi),
        },
        .fork_damper = {
            .core = copy_core(c.fork_damper.core),
            .total_travel_mm = static_cast<Real>(c.fork_damper.total_travel_mm),
            .hbo_start_mm = static_cast<Real>(c.fork_damper.hbo_start_mm),
            .c_hbo_base = static_cast<Real>(c.fork_damper.c_hbo_base),
        },
        .shock_damper = {
            .core = copy_core(c.shock_damper.core),
            .total_stroke_mm = static_cast<Real>(c.shock_damper.total_stroke_mm),
            .max_hbo = c.shock_damper.max_hbo,
            .hbo_clicks = c.shock_damper.hbo_clicks,
            .lockout_firm = static_cast<u8>(c.shock_damper.lockout_firm ? 1 : 0),
            .hbo_start_mm = static_cast<Real>(c.shock_damper.hbo_start_mm),
            .c_hbo_min = static_cast<Real>(c.shock_damper.c_hbo_min),
            .c_hbo_max = static_cast<Real>(c.shock_damper.c_hbo_max),
            .lockout_preload_n = static_cast<Real>(c.shock_damper.lockout_preload_n),
            .lockout_stiffness = static_cast<Real>(c.shock_damper.lockout_stiffness),
        },
        .coil = {
            .rate_n_m = static_cast<Real>(c.coil.rate_n_m),
            .preload_mm = static_cast<Real>(c.coil.preload_mm),
            .stroke_mm = static_cast<Real>(c.coil.stroke_mm),
            .bumper_length_mm = static_cast<Real>(c.coil.bumper_length_mm),
            .bumper_peak_n = static_cast<Real>(c.coil.bumper_peak_n),
        },
        .end_stops = {
            .stiffness_n_m = static_cast<Real>(c.end_stops.stiffness_n_m),
            .damping_n_s_m = static_cast<Real>(c.end_stops.damping_n_s_m),
        },
    };
}

[[gnu::noinline]] void test_logf_and_powf_portable() {
    const float inf_pos = std::numeric_limits<float>::infinity();
    const float nan_q = std::numeric_limits<float>::quiet_NaN();
    const float zero_pos = 0.0f;
    const float zero_neg = -0.0f;
    const float neg_one = -1.0f;

    require(std::isinf(GpuToolkit::log(zero_pos)) && GpuToolkit::log(zero_pos) < 0.0f, "log(+0)");
    require(std::isinf(GpuToolkit::log(zero_neg)) && GpuToolkit::log(zero_neg) < 0.0f, "log(-0)");
    require(std::isnan(GpuToolkit::log(neg_one)), "log(-1)");
    require(std::isnan(GpuToolkit::log(nan_q)), "log(NaN)");
    require(std::isinf(GpuToolkit::log(inf_pos)) && GpuToolkit::log(inf_pos) > 0.0f, "log(+Inf)");
    require_bits_f32(GpuToolkit::log(1.0f), 0.0f, "log(1)");

    // Subnormals and wide positive domain (0, 1e6]
    const std::array<float, 6> edge_pos = {
        1.0e-40f, 1.0e-38f, 0.5f, 0.70710678f, 1.0f, 1.0e6f
    };
    for (const float x : edge_pos) {
        const float g = GpuToolkit::log(x);
        const float ref = static_cast<float>(std::log(static_cast<double>(x)));
        require(float_ulp_diff(g, ref) <= 2u, "logf_portable edge ULP <= 2");
    }

    for (int i = 1; i <= 4000; ++i) {
        const double t = static_cast<double>(i) / 4000.0;
        const float x = static_cast<float>(std::exp(-12.0 + t * (12.0 + std::log(1.0e6))));
        const float g = GpuToolkit::log(x);
        const float ref = static_cast<float>(std::log(static_cast<double>(x)));
        require(float_ulp_diff(g, ref) <= 2u, "logf_portable logspace ULP <= 2");
    }
    for (int i = 0; i <= 2000; ++i) {
        const float x = 0.5f + static_cast<float>(i) * 0.00075f;
        const float g = GpuToolkit::log(x);
        const float ref = static_cast<float>(std::log(static_cast<double>(x)));
        require(float_ulp_diff(g, ref) <= 2u, "logf_portable [0.5,2.0] ULP <= 2");
    }

    // powf_portable special cases
    require_bits_f32(GpuToolkit::pow_py(0.0f, 0.0f), 1.0f, "pow(0,0)");
    require_bits_f32(GpuToolkit::pow_py(nan_q, 0.0f), 1.0f, "pow(NaN,0)");
    require_bits_f32(GpuToolkit::pow_py(1.0f, nan_q), 1.0f, "pow(1,NaN)");
    require_bits_f32(GpuToolkit::pow_py(0.0f, 1.4f), 0.0f, "pow(0, 1.4)");
    require(std::isinf(GpuToolkit::pow_py(0.0f, -1.0f)), "pow(0, -1)");
    require(std::isnan(GpuToolkit::pow_py(-2.0f, 1.4f)), "pow(-2, 1.4)");
    require(std::isnan(GpuToolkit::pow_py(2.0f, nan_q)), "pow(2, NaN)");

    const std::array<float, 10> exps = {
        0.10f, 0.12f, 0.88f, 0.90f, 1.15f, 1.20f, 1.25f, 1.30f, 1.40f, 2.0f
    };
    for (const float e : exps) {
        for (int i = 1; i <= 200; ++i) {
            const float x = static_cast<float>(i) * 0.05f;
            const float g = GpuToolkit::pow_py(x, e);
            const float ref = static_cast<float>(std::pow(static_cast<double>(x), static_cast<double>(e)));
            require(float_ulp_diff(g, ref) <= 4u, "powf_portable [0.05, 10] ULP <= 4");
        }
        // Large domain (0, 1e6] relative error check
        const std::array<float, 4> big_x = {100.0f, 1.0e4f, 1.0e5f, 1.0e6f};
        for (const float x : big_x) {
            const float g = GpuToolkit::pow_py(x, e);
            const double ref = std::pow(static_cast<double>(x), static_cast<double>(e));
            const double rel = std::abs(static_cast<double>(g) - ref) / ref;
            require(rel <= 5.0e-6, "powf_portable (0, 1e6] rel <= 5e-6");
        }
    }
}

[[gnu::noinline]] void test_suspension_parity(const mjModel *m, const mjData *d, std::mt19937_64 &rng) {
    const auto cfg_base = make_stock_suspension_cfg();
    auto cfg_lock = cfg_base;
    cfg_lock.shock_damper.lockout_firm = true;

    const SuspensionWriter sw_base(m, cfg_base);
    const SuspensionWriter sw_lock(m, cfg_lock);
    const auto p64_base = to_core_susp<double>(cfg_base);
    const auto p32_base = to_core_susp<float>(cfg_base);
    const auto p64_lock = to_core_susp<double>(cfg_lock);
    const auto p32_lock = to_core_susp<float>(cfg_lock);

    const auto qpos = model_access::readonly_buffer(d->qpos, m->nq);
    const auto qvel = model_access::readonly_buffer(d->qvel, m->nv);
    const auto qpos_mut = std::views::counted(d->qpos, m->nq);
    const auto qvel_mut = std::views::counted(d->qvel, m->nv);
    (void)qpos;
    (void)qvel;

    const int fork_q = sw_base.fork_qposadr();
    const int fork_v = sw_base.fork_dofadr();
    const int shock_q = sw_base.shock_qposadr();
    const int shock_v = sw_base.shock_dofadr();

    std::uniform_real_distribution<double> dist_fq(-0.005, 0.180);
    std::uniform_real_distribution<double> dist_sq(-0.008, 0.075);
    std::uniform_real_distribution<double> dist_vel(-3.0, 3.0);

    std::array<ForceComponentView, SuspensionWriter::kPhysicalComponentCount> views{};
    double max_err_shock_tot = 0.0;
    double max_err_fork_dmp = 0.0;

    for (int i = 0; i < 4000; ++i) {
        const bool lockout = (i >= 2000);
        double fq = dist_fq(rng);
        double sq = dist_sq(rng);
        double fv = dist_vel(rng);
        double sv = dist_vel(rng);
        if (i == 0) { fq = 0.0; sq = 0.0; fv = 0.0; sv = 0.0; }
        if (i == 1) { fq = 0.160; sq = 0.065; fv = 0.2; sv = 0.2; }
        if (i == 2) { fq = 0.179; sq = 0.072; fv = 2.5; sv = 2.5; }
        if (i == 3) { fq = -0.005; sq = -0.005; fv = -1.5; sv = -1.5; }
        if (i == 4) { fq = 0.165; sq = 0.055; fv = 0.18; sv = 0.16; }

        qpos_mut[static_cast<std::size_t>(fork_q)] = fq;
        qvel_mut[static_cast<std::size_t>(fork_v)] = fv;
        qpos_mut[static_cast<std::size_t>(shock_q)] = sq;
        qvel_mut[static_cast<std::size_t>(shock_v)] = sv;

        const auto &sw = lockout ? sw_lock : sw_base;
        sw.components_into(d, views);

        SuspensionResult<double> r64{};
        ErrCode err64 = ErrCode::none;
        const Status st64 = bike::physics_core::eval_suspension<double, CpuExactToolkit>(
            lockout ? &p64_lock : &p64_base, fq, fv, sq, sv, r64, err64);
        require(st64 == Status::ok, "eval_suspension<double> ok");

        SuspensionResult<float> r32{};
        ErrCode err32 = ErrCode::none;
        const Status st32 = bike::physics_core::eval_suspension<float, GpuToolkit>(
            lockout ? &p32_lock : &p32_base,
            static_cast<float>(fq), static_cast<float>(fv),
            static_cast<float>(sq), static_cast<float>(sv), r32, err32);
        require(st32 == Status::ok, "eval_suspension<float> ok");

        const std::array<double, 8> expected_qfrc{
            -r64.fork_spring,
            -r64.fork_damper,
            -r64.shock_spring,
            -r64.shock_bumper,
            -(r64.shock_damper - r64.shock_hbo),
            -r64.shock_top_out,
            -r64.shock_upper_stop,
            -r64.shock_hbo,
        };
        const std::array<int, 8> dofs{
            fork_v, fork_v, shock_v, shock_v, shock_v, shock_v, shock_v, shock_v
        };
        for (std::size_t c = 0; c < 8; ++c) {
            require_bits_f64(
                views[c].values[static_cast<std::size_t>(dofs[c])],
                expected_qfrc[c],
                "SuspensionWriter 0-bit parity");
        }
        require_bits_f64(sw.shock_total_n(), r64.shock_total, "SuspensionWriter shock_total_n");

        max_err_shock_tot = std::max(
            max_err_shock_tot,
            std::abs(static_cast<double>(r32.shock_total) - r64.shock_total));
        max_err_fork_dmp = std::max(
            max_err_fork_dmp,
            std::abs(static_cast<double>(r32.fork_damper) - r64.fork_damper));
    }
    require(max_err_shock_tot <= 0.05, "FP32 shock_total <= 0.05 N");
    require(max_err_fork_dmp <= 0.015, "FP32 fork_damper <= 0.015 N");

    // Validation & overflow order checks
    {
        SuspensionParams<double> bad_air = p64_base;
        bad_air.air_spring.num_tokens = 4;
        bad_air.air_spring.token_volume_cm3 = 50.0;
        SuspensionResult<double> r{};
        ErrCode err = ErrCode::none;
        const Status st = bike::physics_core::eval_suspension<double, CpuExactToolkit>(
            &bad_air, 0.15, 0.0, 0.0, 0.0, r, err);
        require(st == Status::invalid_argument && err == ErrCode::outside_domain &&
                    r.diag_field == SuspDiagField::air_volume_exhausted,
                "pneumatic volume exhausted status");

        auto cfg_bad = cfg_base;
        cfg_bad.air_spring.num_tokens = 4;
        cfg_bad.air_spring.specs.token_volume_cm3 = 50.0;
        // Bypass validate if needed via direct core test; test Writer throw with large travel + 4 tokens:
        auto cfg_exhaust = cfg_base;
        cfg_exhaust.air_spring.num_tokens = 4;
        const SuspensionWriter sw_exh(m, cfg_exhaust);
        qpos_mut[static_cast<std::size_t>(fork_q)] = 0.175;
        qvel_mut[static_cast<std::size_t>(fork_v)] = 0.0;
        qpos_mut[static_cast<std::size_t>(shock_q)] = 0.0;
        qvel_mut[static_cast<std::size_t>(shock_v)] = 0.0;
        // Verify +Inf fork velocity throws overflow_error through SuspensionWriter:
        qpos_mut[static_cast<std::size_t>(fork_q)] = 0.05;
        qvel_mut[static_cast<std::size_t>(fork_v)] = std::numeric_limits<double>::infinity();
        bool caught_overflow = false;
        try {
            sw_exh.components_into(d, views);
        } catch (const std::overflow_error &) {
            caught_overflow = true;
        }
        require(caught_overflow, "SuspensionWriter +Inf fork velocity throws overflow_error");

        SuspensionParams<double> zero_hbo = p64_base;
        zero_hbo.fork_damper.hbo_start_mm = zero_hbo.fork_damper.total_travel_mm;
        require(bike::physics_core::eval_suspension<double, CpuExactToolkit>(
                    &zero_hbo, 0.185, 1.0, 0.0, 0.0, r, err) == Status::invalid_argument &&
                    r.diag_field == SuspDiagField::charger3_zero_hbo_denom,
                "Charger3 zero HBO denominator");

        require(bike::physics_core::eval_suspension<double, CpuExactToolkit>(
                    &p64_base, 0.05, 0.0, 0.02, std::numeric_limits<double>::quiet_NaN(), r, err) ==
                        Status::invalid_argument &&
                    r.diag_field == SuspDiagField::sd_nonfinite_state,
                "SuperDeluxe nonfinite state");
    }
}

[[gnu::noinline]] void test_resistance_parity(const mjModel *m, mjData *d, std::mt19937_64 &rng) {
    nativecfg::ResistanceConfig rcfg{};
    rcfg.crr = 0.015;
    rcfg.rolling_taper_rad_s = 0.2;
    rcfg.rho_kg_m3 = 1.2;
    rcfg.cda_m2 = 0.5;
    rcfg.wind_world_mps = {0.5, 0.0, 0.0};
    rcfg.point_body_m = {0.0, 0.0, 0.5};
    rcfg.frame_body = "frame";
    rcfg.front_wheel_body = "front_wheel";
    rcfg.rear_wheel_body = "rear_wheel";

    const ResistanceWriter rw(m, rcfg);
    const int frame_id = mj_name2id(m, mjOBJ_BODY, "frame");
    const int front_id = mj_name2id(m, mjOBJ_BODY, "front_wheel");
    const int rear_id = mj_name2id(m, mjOBJ_BODY, "rear_wheel");
    require(frame_id > 0 && front_id > 0 && rear_id > 0, "welded body ids");

    const std::array<GpuWheelChain, 2> wheel_chains = {
        bike::physics_core::build_wheel_chain(m, front_id),
        bike::physics_core::build_wheel_chain(m, rear_id),
    };
    const GpuWheelChain frame_chain = bike::physics_core::build_wheel_chain(m, frame_id);

    const ResistanceParams<double> rp64{
        .crr = rcfg.crr,
        .rolling_taper_rad_s = rcfg.rolling_taper_rad_s,
        .rho_kg_m3 = rcfg.rho_kg_m3,
        .cda_m2 = rcfg.cda_m2,
        .wind_world_mps = {rcfg.wind_world_mps[0], rcfg.wind_world_mps[1], rcfg.wind_world_mps[2]},
        .point_body_m = {rcfg.point_body_m[0], rcfg.point_body_m[1], rcfg.point_body_m[2]},
    };
    const ResistanceParams<float> rp32{
        .crr = static_cast<float>(rcfg.crr),
        .rolling_taper_rad_s = static_cast<float>(rcfg.rolling_taper_rad_s),
        .rho_kg_m3 = static_cast<float>(rcfg.rho_kg_m3),
        .cda_m2 = static_cast<float>(rcfg.cda_m2),
        .wind_world_mps = {static_cast<float>(rcfg.wind_world_mps[0]), 0.0f, 0.0f},
        .point_body_m = {0.0f, 0.0f, static_cast<float>(rcfg.point_body_m[2])},
    };

    const i32 nv_i32 = static_cast<i32>(m->nv);
    const i32 njnt_i32 = static_cast<i32>(m->njnt);
    const std::size_t nv = static_cast<std::size_t>(m->nv);
    std::vector<double> jr64(3u * nv), jp64(3u * nv), row64(nv), r_out64(nv), a_out64(nv);
    std::vector<float> body_xpos32(3u * static_cast<std::size_t>(m->nbody));
    std::vector<float> jnt_xa32(3u * static_cast<std::size_t>(m->njnt));
    std::vector<float> jnt_xx32(3u * static_cast<std::size_t>(m->njnt));
    std::vector<float> qv32(nv), jr32(3u * nv), jp32(3u * nv), row32(nv), r_out32(nv), a_out32(nv);

    std::array<ForceComponentView, ResistanceWriter::kComponentCount> views{};
    std::uniform_real_distribution<double> d_vel(-20.0, 20.0);
    std::uniform_real_distribution<double> d_pitch(-0.35, 0.35);
    std::uniform_real_distribution<double> d_load(0.0, 2000.0);

    const auto jnt_qp = model_access::readonly_buffer(m->jnt_qposadr, m->njnt);
    const auto jnt_dof = model_access::readonly_buffer(m->jnt_dofadr, m->njnt);
    const int qp_fork = jnt_qp[static_cast<std::size_t>(mj_name2id(m, mjOBJ_JOINT, "fork_travel"))];
    const int qp_main = jnt_qp[static_cast<std::size_t>(mj_name2id(m, mjOBJ_JOINT, "main_pivot"))];
    const int dof_fw = jnt_dof[static_cast<std::size_t>(mj_name2id(m, mjOBJ_JOINT, "front_wheel_spin"))];
    const int dof_rw = jnt_dof[static_cast<std::size_t>(mj_name2id(m, mjOBJ_JOINT, "rear_wheel_spin"))];
    const int dof_mp = jnt_dof[static_cast<std::size_t>(mj_name2id(m, mjOBJ_JOINT, "main_pivot"))];
    const int dof_hp = jnt_dof[static_cast<std::size_t>(mj_name2id(m, mjOBJ_JOINT, "horst_pivot"))];
    const int dof_pitch = jnt_dof[static_cast<std::size_t>(mj_name2id(m, mjOBJ_JOINT, "root_pitch"))];

    double max_err_roll = 0.0;
    double max_err_aero = 0.0;

    for (int i = 0; i < 1000; ++i) {
        engine::reset_data(m, d);
        const auto qpos_mut = std::views::counted(d->qpos, m->nq);
        const auto qvel_mut = std::views::counted(d->qvel, m->nv);
        qpos_mut[2] = d_pitch(rng);
        qpos_mut[static_cast<std::size_t>(qp_fork)] = 0.05;
        qpos_mut[static_cast<std::size_t>(qp_main)] = 0.08;
        qvel_mut[0] = d_vel(rng);
        qvel_mut[1] = 0.3 * d_vel(rng);
        qvel_mut[2] = (i % 10) * 0.2 - 1.0;
        qvel_mut[static_cast<std::size_t>(dof_fw)] = d_vel(rng) / 0.36;
        qvel_mut[static_cast<std::size_t>(dof_rw)] = d_vel(rng) / 0.34;
        qvel_mut[static_cast<std::size_t>(dof_mp)] = 0.5;
        qvel_mut[static_cast<std::size_t>(dof_hp)] = -0.3;
        engine::forward(m, d);

        const std::size_t n_patches = (i % 2 == 0) ? 128u : 1u;
        std::vector<double> f_loads(n_patches), r_loads(n_patches);
        std::vector<float> f_loads32(n_patches), r_loads32(n_patches);
        std::array<bool, 128> f_work_b{}, r_work_b{};
        std::array<u8, 128> f_work_u{}, r_work_u{};
        for (std::size_t p = 0; p < n_patches; ++p) {
            f_loads[p] = d_load(rng) / static_cast<double>(n_patches);
            r_loads[p] = d_load(rng) / static_cast<double>(n_patches);
            f_loads32[p] = static_cast<float>(f_loads[p]);
            r_loads32[p] = static_cast<float>(r_loads[p]);
            f_work_b[p] = (p % 7 != 0) || n_patches == 1u;
            r_work_b[p] = (p % 5 != 0) || n_patches == 1u;
            f_work_u[p] = f_work_b[p] ? 1u : 0u;
            r_work_u[p] = r_work_b[p] ? 1u : 0u;
        }
        const double f_eff = 0.365;
        const double r_eff = 0.345;
        const TireSideInput front_in{
            .patch_loads = f_loads,
            .patch_working = std::span<const bool>(f_work_b).first(n_patches),
            .effective_radius_m = f_eff};
        const TireSideInput rear_in{
            .patch_loads = r_loads,
            .patch_working = std::span<const bool>(r_work_b).first(n_patches),
            .effective_radius_m = r_eff};
        rw.components_into(d, front_in, rear_in, views);

        const auto xpos = model_access::readonly_buffer(d->xpos, 3 * m->nbody);
        const auto xmat = model_access::readonly_buffer(d->xmat, 9 * m->nbody);
        const auto qvel = model_access::readonly_buffer(d->qvel, m->nv);
        const auto xanchor = model_access::readonly_buffer(d->xanchor, 3 * m->njnt);
        const auto xaxis = model_access::readonly_buffer(d->xaxis, 3 * m->njnt);

        auto wheel_jr_cpu = [&](i32 side, const double *wxpos, double *out_jr, ErrCode &) noexcept {
            const int body = (side == 0) ? front_id : rear_id;
            mj_jac(m, d, nullptr, out_jr, wxpos, body);
            return Status::ok;
        };
        auto frame_jp_cpu = [&](const double *pt, double *out_jp, ErrCode &) noexcept {
            mj_jac(m, d, out_jp, nullptr, pt, frame_id);
            return Status::ok;
        };

        ErrCode err64 = ErrCode::none;
        ResDiagField diag64 = ResDiagField::none;
        const Status st64 = bike::physics_core::eval_resistance<double, CpuExactToolkit>(
            nv_i32, &rp64, qvel.data(),
            f_loads.data(), f_work_b.data(), static_cast<i32>(n_patches), f_eff,
            &xpos[3u * static_cast<std::size_t>(front_id)],
            &xmat[9u * static_cast<std::size_t>(front_id)],
            r_loads.data(), r_work_b.data(), static_cast<i32>(n_patches), r_eff,
            &xpos[3u * static_cast<std::size_t>(rear_id)],
            &xmat[9u * static_cast<std::size_t>(rear_id)],
            &xpos[3u * static_cast<std::size_t>(frame_id)],
            &xmat[9u * static_cast<std::size_t>(frame_id)],
            jr64.data(), jp64.data(), row64.data(),
            r_out64.data(), a_out64.data(),
            wheel_jr_cpu, frame_jp_cpu, err64, diag64);
        require(st64 == Status::ok, "eval_resistance<double> ok");

        for (std::size_t k = 0; k < nv; ++k) {
            require_bits_f64(views[0].values[k], r_out64[k], "ResistanceWriter rolling 0-bit");
            require_bits_f64(views[1].values[k], a_out64[k], "ResistanceWriter aero 0-bit");
        }
        if (i == 0) {
            require(std::abs(r_out64[static_cast<std::size_t>(dof_pitch)]) > 1e-6,
                    "rolling couples into root_pitch");
            require(std::abs(r_out64[static_cast<std::size_t>(dof_mp)]) > 1e-6,
                    "rolling couples into main_pivot");
            require(std::abs(r_out64[static_cast<std::size_t>(dof_hp)]) > 1e-6,
                    "rolling couples into horst_pivot");
            require(std::abs(a_out64[static_cast<std::size_t>(dof_pitch)]) > 1e-3,
                    "aero induces root_pitch moment");
        }

        // Test GPU FP32 with analytical ChainWheelJrFn + ChainFrameJpFn!
        for (std::size_t k = 0; k < body_xpos32.size(); ++k) body_xpos32[k] = static_cast<float>(xpos[k]);
        for (std::size_t k = 0; k < jnt_xa32.size(); ++k) jnt_xa32[k] = static_cast<float>(xanchor[k]);
        for (std::size_t k = 0; k < jnt_xx32.size(); ++k) jnt_xx32[k] = static_cast<float>(xaxis[k]);
        for (std::size_t k = 0; k < nv; ++k) qv32[k] = static_cast<float>(qvel[k]);

        std::array<float, 3> fx32{}, rx32{}, frx32{};
        std::array<float, 9> fm32{}, rm32{}, frm32{};
        for (std::size_t k = 0; k < 3; ++k) {
            fx32[k] = static_cast<float>(xpos[static_cast<std::size_t>(3 * front_id) + k]);
            rx32[k] = static_cast<float>(xpos[static_cast<std::size_t>(3 * rear_id) + k]);
            frx32[k] = static_cast<float>(xpos[static_cast<std::size_t>(3 * frame_id) + k]);
        }
        for (std::size_t k = 0; k < 9; ++k) {
            fm32[k] = static_cast<float>(xmat[static_cast<std::size_t>(9 * front_id) + k]);
            rm32[k] = static_cast<float>(xmat[static_cast<std::size_t>(9 * rear_id) + k]);
            frm32[k] = static_cast<float>(xmat[static_cast<std::size_t>(9 * frame_id) + k]);
        }

        const ChainWheelJrFn<float, GpuToolkit, const float *, const GpuWheelChain *> wheel_jr_gpu{
            .chains = wheel_chains.data(),
            .jnt_xaxis = jnt_xx32.data(),
            .nv = nv_i32,
            .njnt = njnt_i32};
        const ChainFrameJpFn<float, GpuToolkit, const float *, const float *, const float *> frame_jp_gpu{
            .chain = frame_chain,
            .body_xpos = body_xpos32.data(),
            .jnt_xanchor = jnt_xa32.data(),
            .jnt_xaxis = jnt_xx32.data(),
            .nv = nv_i32,
            .njnt = njnt_i32};

        ErrCode err32 = ErrCode::none;
        ResDiagField diag32 = ResDiagField::none;
        const Status st32 = bike::physics_core::eval_resistance<float, GpuToolkit>(
            nv_i32, &rp32, qv32.data(),
            f_loads32.data(), f_work_u.data(), static_cast<i32>(n_patches), static_cast<float>(f_eff),
            fx32.data(), fm32.data(),
            r_loads32.data(), r_work_u.data(), static_cast<i32>(n_patches), static_cast<float>(r_eff),
            rx32.data(), rm32.data(),
            frx32.data(), frm32.data(),
            jr32.data(), jp32.data(), row32.data(),
            r_out32.data(), a_out32.data(),
            wheel_jr_gpu, frame_jp_gpu, err32, diag32);
        require(st32 == Status::ok, "eval_resistance<float> w/ kinematics ok");

        for (std::size_t k = 0; k < nv; ++k) {
            max_err_roll = std::max(max_err_roll, std::abs(static_cast<double>(r_out32[k]) - r_out64[k]));
            max_err_aero = std::max(max_err_aero, std::abs(static_cast<double>(a_out32[k]) - a_out64[k]));
        }
    }
    require(max_err_roll <= 5.0e-5, "FP32 rolling_qfrc <= 5e-5");
    require(max_err_aero <= 5.0e-5, "FP32 aero_qfrc <= 5e-5");
}

[[gnu::noinline]] void test_brake_hold_parity(const mjModel *m, const mjData *d, std::mt19937_64 &rng) {
    const auto jnt_dof = model_access::readonly_buffer(m->jnt_dofadr, m->njnt);
    const auto jnt_qp = model_access::readonly_buffer(m->jnt_qposadr, m->njnt);
    const int f_dof = jnt_dof[static_cast<std::size_t>(mj_name2id(m, mjOBJ_JOINT, "front_wheel_spin"))];
    const int r_dof = jnt_dof[static_cast<std::size_t>(mj_name2id(m, mjOBJ_JOINT, "rear_wheel_spin"))];
    const int f_qp = jnt_qp[static_cast<std::size_t>(mj_name2id(m, mjOBJ_JOINT, "front_wheel_spin"))];
    const int r_qp = jnt_qp[static_cast<std::size_t>(mj_name2id(m, mjOBJ_JOINT, "rear_wheel_spin"))];

    const runtime::BrakeHoldSpec hspec{
        .torque_nm = 200.0,
        .stiffness_nm_rad = 20000.0,
        .damping_nms_rad = 10.0,
        .engage_radps = 0.5,
        .release_radps = 1.0,
    };
    runtime::StaticBrake sb(f_dof, r_dof, 200.0, hspec);
    const BrakeHoldParams<double> bp64{
        .torque_nm = 200.0,
        .stiffness_nm_rad = 20000.0,
        .damping_nms_rad = 10.0,
        .engage_radps = 0.5,
        .release_radps = 1.0,
    };
    const BrakeHoldParams<float> bp32{
        .torque_nm = 200.0f,
        .stiffness_nm_rad = 20000.0f,
        .damping_nms_rad = 10.0f,
        .engage_radps = 0.5f,
        .release_radps = 1.0f,
    };

    std::uniform_real_distribution<double> d_creep(-0.005, 0.005);
    std::uniform_real_distribution<double> d_omega(-0.6, 0.6);

    double max_near0 = 0.0;
    double max_q5000_raw = 0.0;
    double max_q5000_rebased_mod = 0.0;
    double max_q5000_rebased_turn = 0.0;

    const auto qpos_mut = std::views::counted(d->qpos, m->nq);
    const auto qvel_mut = std::views::counted(d->qvel, m->nv);

    for (int regime = 0; regime < 4; ++regime) {
        const double q_base = (regime == 0)
            ? 0.0
            : (regime == 3 ? 796.0 * 2.0 * std::numbers::pi : 5000.0);
        sb.reset();
        std::array<bool, 2> has64{false, false};
        std::array<bool, 2> has32{false, false};
        std::array<double, 2> latch64{0.0, 0.0};
        std::array<float, 2> latch32{0.0f, 0.0f};

        for (int step = 0; step < 500; ++step) {
            const double th_f = q_base + (step == 0 ? 0.0 : d_creep(rng));
            const double th_r = q_base + (step == 0 ? 0.0 : d_creep(rng));
            const double w_f = (step == 0) ? 0.0 : d_omega(rng);
            const double w_r = (step == 0) ? 0.0 : d_omega(rng);
            const double dem_f = (step % 50 == 49) ? 0.0 : 1.0;
            const double dem_r = (step % 50 == 49) ? 0.0 : 0.8;
            const bool adv = (step % 3 != 2);
            const bool res = (step % 10 == 5);

            qpos_mut[static_cast<std::size_t>(f_qp)] = th_f;
            qpos_mut[static_cast<std::size_t>(r_qp)] = th_r;
            qvel_mut[static_cast<std::size_t>(f_dof)] = w_f;
            qvel_mut[static_cast<std::size_t>(r_dof)] = w_r;

            const auto v = sb.hold_qfrc(m, d, dem_f, dem_r, adv, res);

            double out64_f = 0.0;
            double out64_r = 0.0;
            ErrCode e64 = ErrCode::none;
            require(bike::physics_core::eval_brake_hold_wheel<double, CpuExactToolkit>(
                        &bp64, dem_f, th_f, w_f, adv, res, has64[0], latch64[0], out64_f, e64) == Status::ok,
                    "eval_brake_hold_wheel<double> front");
            require(bike::physics_core::eval_brake_hold_wheel<double, CpuExactToolkit>(
                        &bp64, dem_r, th_r, w_r, adv, res, has64[1], latch64[1], out64_r, e64) == Status::ok,
                    "eval_brake_hold_wheel<double> rear");

            require_bits_f64(v[static_cast<std::size_t>(f_dof)], out64_f, "StaticBrake front 0-bit");
            require_bits_f64(v[static_cast<std::size_t>(r_dof)], out64_r, "StaticBrake rear 0-bit");

            double reb_f = th_f;
            double reb_r = th_r;
            if (regime >= 2) {
                double dummy_latch_f = 0.0;
                double dummy_latch_r = 0.0;
                const double sf = bike::physics_core::rebase_2pi_angle<double, CpuExactToolkit>(
                    reb_f, false, dummy_latch_f);
                const double sr = bike::physics_core::rebase_2pi_angle<double, CpuExactToolkit>(
                    reb_r, false, dummy_latch_r);
                (void)sf;
                (void)sr;
            }
            float out32_f = 0.0f;
            float out32_r = 0.0f;
            ErrCode e32 = ErrCode::none;
            require(bike::physics_core::eval_brake_hold_wheel<float, GpuToolkit>(
                        &bp32, static_cast<float>(dem_f), static_cast<float>(reb_f),
                        static_cast<float>(w_f), adv, res, has32[0], latch32[0], out32_f, e32) == Status::ok,
                    "eval_brake_hold_wheel<float> front");
            require(bike::physics_core::eval_brake_hold_wheel<float, GpuToolkit>(
                        &bp32, static_cast<float>(dem_r), static_cast<float>(reb_r),
                        static_cast<float>(w_r), adv, res, has32[1], latch32[1], out32_r, e32) == Status::ok,
                    "eval_brake_hold_wheel<float> rear");

            const double err_f = std::abs(static_cast<double>(out32_f) - out64_f);
            const double err_r = std::abs(static_cast<double>(out32_r) - out64_r);
            if (regime == 0) {
                max_near0 = std::max({max_near0, err_f, err_r});
            } else if (regime == 1) {
                max_q5000_raw = std::max({max_q5000_raw, err_f, err_r});
            } else if (regime == 2) {
                max_q5000_rebased_mod = std::max({max_q5000_rebased_mod, err_f, err_r});
            } else {
                max_q5000_rebased_turn = std::max({max_q5000_rebased_turn, err_f, err_r});
            }
        }
    }
    require(max_near0 <= 5.0e-5, "brake_hold q~0 FP32 <= 5e-5");
    require(max_q5000_raw > 1.0, "brake_hold q=5000 raw cancellation observed");
    require(max_q5000_rebased_mod <= 3.0e-3, "brake_hold q=5000 arbitrary phase rebased <= 3e-3");
    require(max_q5000_rebased_turn <= 5.0e-5, "brake_hold q~5000 (796*2pi) rebased FP32 <= 5e-5");
}

[[gnu::noinline]] void test_joint_envelope_parity(const mjModel *m, std::mt19937_64 &rng) {
    const auto limited = model_access::readonly_buffer(m->jnt_limited, m->njnt);
    const auto range = model_access::readonly_buffer(m->jnt_range, 2 * m->njnt);
    const auto qp_adr = model_access::readonly_buffer(m->jnt_qposadr, m->njnt);
    const auto dof_adr = model_access::readonly_buffer(m->jnt_dofadr, m->njnt);

    RiderJointEnvelopeSpec<double> spec64{};
    RiderJointEnvelopeSpec<float> spec32{};
    spec64.stiffness_nm_rad = 100.0;
    spec64.margin_rad = 0.1;
    spec32.stiffness_nm_rad = 100.0f;
    spec32.margin_rad = 0.1f;

    const auto s64_qp = std::span(spec64.qpos_adrs);
    const auto s64_dof = std::span(spec64.dof_adrs);
    const auto s64_lo = std::span(spec64.lower);
    const auto s64_hi = std::span(spec64.upper);
    const auto s32_qp = std::span(spec32.qpos_adrs);
    const auto s32_dof = std::span(spec32.dof_adrs);
    const auto s32_lo = std::span(spec32.lower);
    const auto s32_hi = std::span(spec32.upper);

    for (int j = 0; j < m->njnt; ++j) {
        if (limited[static_cast<std::size_t>(j)] == 0) {
            continue;
        }
        const char *name = mj_id2name(m, mjOBJ_JOINT, j);
        if (name == nullptr || !std::string_view(name).starts_with("rider_")) {
            continue;
        }
        const std::size_t idx = static_cast<std::size_t>(spec64.count++);
        spec32.count = spec64.count;
        s64_qp[idx] = qp_adr[static_cast<std::size_t>(j)];
        s64_dof[idx] = dof_adr[static_cast<std::size_t>(j)];
        s64_lo[idx] = range[2u * static_cast<std::size_t>(j)];
        s64_hi[idx] = range[2u * static_cast<std::size_t>(j) + 1u];

        s32_qp[idx] = s64_qp[idx];
        s32_dof[idx] = s64_dof[idx];
        s32_lo[idx] = static_cast<float>(s64_lo[idx]);
        s32_hi[idx] = static_cast<float>(s64_hi[idx]);
    }
    require(spec64.count == 9, "welded.xml has 9 limited rider joints");

    std::vector<double> qpos64(static_cast<std::size_t>(m->nq), 0.0);
    std::vector<float> qpos32(static_cast<std::size_t>(m->nq), 0.0f);
    std::vector<double> qfrc64(static_cast<std::size_t>(m->nv), 0.0);
    std::vector<double> qfrc_ref(static_cast<std::size_t>(m->nv), 0.0);
    std::vector<float> qfrc32(static_cast<std::size_t>(m->nv), 0.0f);

    double max_tau_err = 0.0;
    double max_eng_err = 0.0;

    for (int s = 0; s < 500; ++s) {
        std::ranges::fill(qfrc64, 0.0);
        std::ranges::fill(qfrc_ref, 0.0);
        std::ranges::fill(qfrc32, 0.0f);
        double stored_ref = 0.0;

        for (i32 i = 0; i < spec64.count; ++i) {
            const std::size_t ui = static_cast<std::size_t>(i);
            std::uniform_real_distribution<double> dq(
                s64_lo[ui] - 0.15, s64_hi[ui] + 0.15);
            const double q = dq(rng);
            qpos64[static_cast<std::size_t>(s64_qp[ui])] = q;
            qpos32[static_cast<std::size_t>(s64_qp[ui])] = static_cast<float>(q);

            const auto ref = spindle::soft_edge_scalar(
                q, s64_lo[ui], s64_hi[ui], 100.0, 0.1);
            qfrc_ref[static_cast<std::size_t>(s64_dof[ui])] += ref.torque;
            stored_ref += ref.stored_j;

            bike::physics_core::SoftEdgeResult<double> r64{};
            ErrCode err64 = ErrCode::none;
            require(bike::physics_core::soft_edge_scalar<double, CpuExactToolkit>(
                        q, s64_lo[ui], s64_hi[ui], 100.0, 0.1, r64, err64) == Status::ok,
                    "soft_edge_scalar<double>");
            require_bits_f64(ref.torque, r64.torque, "soft_edge_scalar torque 0-bit");
            require_bits_f64(ref.stored_j, r64.stored_j, "soft_edge_scalar stored 0-bit");
        }

        double stored64 = 0.0;
        ErrCode err64 = ErrCode::none;
        const i32 nq_i32 = static_cast<i32>(m->nq);
        const i32 nv_i32 = static_cast<i32>(m->nv);
        require(bike::physics_core::eval_rider_joint_envelope<double, CpuExactToolkit>(
                    &spec64, nq_i32, nv_i32, qpos64.data(), qfrc64.data(), stored64, err64) == Status::ok,
                "eval_rider_joint_envelope<double>");
        require_bits_f64(stored64, stored_ref, "envelope stored_j 0-bit");
        for (std::size_t k = 0; k < qfrc64.size(); ++k) {
            require_bits_f64(qfrc64[k], qfrc_ref[k], "envelope qfrc 0-bit");
        }

        float stored32 = 0.0f;
        ErrCode err32 = ErrCode::none;
        require(bike::physics_core::eval_rider_joint_envelope<float, GpuToolkit>(
                    &spec32, nq_i32, nv_i32, qpos32.data(), qfrc32.data(), stored32, err32) == Status::ok,
                "eval_rider_joint_envelope<float>");
        max_eng_err = std::max(max_eng_err, std::abs(static_cast<double>(stored32) - stored64));
        for (std::size_t k = 0; k < qfrc64.size(); ++k) {
            max_tau_err = std::max(
                max_tau_err, std::abs(static_cast<double>(qfrc32[k]) - qfrc64[k]));
        }
    }
    require(max_tau_err <= 5.0e-5, "envelope FP32 torque <= 5e-5");
    require(max_eng_err <= 5.0e-5, "envelope FP32 energy <= 5e-5");
}

[[gnu::noinline]] void test_ratchet_parity(const std::filesystem::path &xml_path, std::mt19937_64 &rng) {
    const std::array<int, 12> cassette{10, 12, 14, 16, 18, 21, 24, 28, 33, 39, 45, 51};

    ModelPtr m_copy = load_xml_model(xml_path);
    DataPtr d_copy(engine::make_data(m_copy.get()), delete_data);
    drivetrain::Transmission tx_fh(
        m_copy.get(),
        drivetrain::GearingConfig{.front_teeth = 22, .rear_teeth = 51, .chain_pitch_m = 0.0127},
        false, "ideal_mid_drive_freehub", "drive_shaft_spin", "rear_wheel_spin", "crank");
    drivetrain::Transmission tx_cl(
        m_copy.get(),
        drivetrain::GearingConfig{.front_teeth = 3, .rear_teeth = 3, .chain_pitch_m = 0.0127},
        false, "crank_clutch", "crank_spin", "drive_shaft_spin", "crank");

    const auto jnt_qp = model_access::readonly_buffer(m_copy->jnt_qposadr, m_copy->njnt);
    const int qp_c = jnt_qp[static_cast<std::size_t>(mj_name2id(m_copy.get(), mjOBJ_JOINT, "crank_spin"))];
    const int qp_s = jnt_qp[static_cast<std::size_t>(mj_name2id(m_copy.get(), mjOBJ_JOINT, "drive_shaft_spin"))];
    const int qp_w = jnt_qp[static_cast<std::size_t>(mj_name2id(m_copy.get(), mjOBJ_JOINT, "rear_wheel_spin"))];

    tx_fh.reset(d_copy.get());
    tx_cl.reset(d_copy.get());

    const auto qpos_ro = model_access::readonly_buffer(d_copy->qpos, m_copy->nq);
    const auto qpos_mut = std::views::counted(d_copy->qpos, m_copy->nq);

    double fh_ratio = 22.0 / 51.0;
    bool has_fh = false;
    bool has_cl = false;
    double b_fh = 0.0;
    double b_cl = 0.0;
    ErrCode err64 = ErrCode::none;
    require(bike::physics_core::ratchet_reset_bound<double, CpuExactToolkit>(
                fh_ratio, qpos_ro[static_cast<std::size_t>(qp_s)],
                qpos_ro[static_cast<std::size_t>(qp_w)], has_fh, b_fh, err64) == Status::ok,
            "ratchet_reset_bound freehub");
    require(bike::physics_core::ratchet_reset_bound<double, CpuExactToolkit>(
                1.0, qpos_ro[static_cast<std::size_t>(qp_c)],
                qpos_ro[static_cast<std::size_t>(qp_s)], has_cl, b_cl, err64) == Status::ok,
            "ratchet_reset_bound clutch");

    std::uniform_real_distribution<double> dq(-0.02, 0.04);
    for (int step = 0; step < 200; ++step) {
        qpos_mut[static_cast<std::size_t>(qp_c)] += dq(rng);
        qpos_mut[static_cast<std::size_t>(qp_s)] += dq(rng);
        qpos_mut[static_cast<std::size_t>(qp_w)] += dq(rng);
        tx_fh.prepare(d_copy.get());
        tx_cl.prepare(d_copy.get());

        require(bike::physics_core::ratchet_prepare_bound<double, CpuExactToolkit>(
                    fh_ratio, qpos_ro[static_cast<std::size_t>(qp_s)],
                    qpos_ro[static_cast<std::size_t>(qp_w)], has_fh, b_fh, err64) == Status::ok,
                "ratchet_prepare_bound freehub");
        require(bike::physics_core::ratchet_prepare_bound<double, CpuExactToolkit>(
                    1.0, qpos_ro[static_cast<std::size_t>(qp_c)],
                    qpos_ro[static_cast<std::size_t>(qp_s)], has_cl, b_cl, err64) == Status::ok,
                "ratchet_prepare_bound clutch");

        require(tx_fh.state().boundary.has_value() && tx_cl.state().boundary.has_value(),
                "Transmission boundary engaged");
        require_bits_f64(tx_fh.state().boundary.value_or(0.0), b_fh, "Transmission freehub 0-bit");
        require_bits_f64(tx_cl.state().boundary.value_or(0.0), b_cl, "Transmission clutch 0-bit");

        if (step % 25 == 24) {
            const double nr =
                22.0 / static_cast<double>(cassette[static_cast<std::size_t>((step / 25) % 12)]);
            tx_fh.set_ratio(d_copy.get(), nr);
            require(bike::physics_core::ratchet_shift_ratio_bound<double, CpuExactToolkit>(
                        fh_ratio, nr, qpos_ro[static_cast<std::size_t>(qp_s)],
                        qpos_ro[static_cast<std::size_t>(qp_w)], has_fh, b_fh, err64) == Status::ok,
                    "ratchet_shift_ratio_bound");
            fh_ratio = nr;
            require(tx_fh.state().boundary.has_value(), "Transmission boundary after shift");
            require_bits_f64(tx_fh.state().boundary.value_or(0.0), b_fh, "Transmission set_ratio 0-bit");
        }
    }

    // Cassette range filler check
    {
        std::array<i32, kCassetteCogCount> tids{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
        std::array<float, 24> ranges{};
        ErrCode err = ErrCode::none;
        require(bike::physics_core::fill_cassette_tendon_ranges<float, GpuToolkit>(
                    5, kCassetteCogCount, kCassetteCogCount, -0.125f,
                    tids.data(), ranges.data(), err) == Status::ok,
                "fill_cassette_tendon_ranges ok");
        for (i32 i = 0; i < kCassetteCogCount; ++i) {
            require_bits_f32(ranges[2u * static_cast<std::size_t>(i)], -1.0e12f, "cassette lo");
            const float expected_hi = (i == 5) ? -0.125f : 1.0e12f;
            require_bits_f32(ranges[2u * static_cast<std::size_t>(i) + 1u], expected_hi, "cassette hi");
        }
    }

    // 2000 FP32 precision trials at q = 5000 rad w/ and w/o synchronized 2*pi rebase
    std::uniform_real_distribution<double> d_def(-0.002, 0.002);
    double max_fh_raw = 0.0;
    double max_fh_reb = 0.0;
    double max_cl_raw = 0.0;
    double max_cl_reb = 0.0;

    for (int trial = 0; trial < 2000; ++trial) {
        const int cog = cassette[static_cast<std::size_t>(trial % 12)];
        const double r64 = 22.0 / static_cast<double>(cog);
        const float r32 = 22.0f / static_cast<float>(cog);

        const double qw = 5000.0;
        const double qs = qw / r64 + d_def(rng);
        const double qc = qs + d_def(rng);
        const double qw2 = qw + 0.05;
        const double qs2 = qs + 0.05 / r64 + 0.000136;
        const double qc2 = qc + 0.05 / r64 + 0.0002;

        bool h_fh64 = false, h_cl64 = false;
        double b_fh64 = 0.0, b_cl64 = 0.0;
        (void)bike::physics_core::ratchet_reset_bound<double, CpuExactToolkit>(
            r64, qs, qw, h_fh64, b_fh64, err64);
        (void)bike::physics_core::ratchet_reset_bound<double, CpuExactToolkit>(
            1.0, qc, qs, h_cl64, b_cl64, err64);
        (void)bike::physics_core::ratchet_prepare_bound<double, CpuExactToolkit>(
            r64, qs2, qw2, h_fh64, b_fh64, err64);
        (void)bike::physics_core::ratchet_prepare_bound<double, CpuExactToolkit>(
            1.0, qc2, qs2, h_cl64, b_cl64, err64);
        const double gap64 = (r64 * qs2 - qw2) - b_fh64;
        const double cgap64 = (qc2 - qs2) - b_cl64;

        // Raw FP32 without rebase
        bool h_fh32 = false, h_cl32 = false;
        float b_fh32 = 0.0f, b_cl32 = 0.0f;
        ErrCode err32 = ErrCode::none;
        (void)bike::physics_core::ratchet_reset_bound<float, GpuToolkit>(
            r32, static_cast<float>(qs), static_cast<float>(qw), h_fh32, b_fh32, err32);
        (void)bike::physics_core::ratchet_reset_bound<float, GpuToolkit>(
            1.0f, static_cast<float>(qc), static_cast<float>(qs), h_cl32, b_cl32, err32);
        (void)bike::physics_core::ratchet_prepare_bound<float, GpuToolkit>(
            r32, static_cast<float>(qs2), static_cast<float>(qw2), h_fh32, b_fh32, err32);
        (void)bike::physics_core::ratchet_prepare_bound<float, GpuToolkit>(
            1.0f, static_cast<float>(qc2), static_cast<float>(qs2), h_cl32, b_cl32, err32);
        const float gap32 = (r32 * static_cast<float>(qs2) - static_cast<float>(qw2)) - b_fh32;
        const float cgap32 = (static_cast<float>(qc2) - static_cast<float>(qs2)) - b_cl32;
        max_fh_raw = std::max(max_fh_raw, std::abs(static_cast<double>(gap32) - gap64));
        max_cl_raw = std::max(max_cl_raw, std::abs(static_cast<double>(cgap32) - cgap64));

        // Synchronized 2*pi rebase (in double epoch state before FP32 step, mirroring periodic rebase)
        WorldRotationsRebaseState<double> reb{};
        reb.front_wheel_qpos = qw;
        reb.rear_wheel_qpos = qw;
        reb.crank_qpos = qc;
        reb.drive_shaft_qpos = qs;
        reb.freehub_ratio = r64;
        reb.freehub_boundary = b_fh64;
        reb.clutch_boundary = b_cl64;
        reb.brake_latch[0] = qw - 0.001;
        reb.brake_latch[1] = qw + 0.002;
        reb.has_freehub_boundary = 1;
        reb.has_clutch_boundary = 1;
        reb.has_brake_latch[0] = 1;
        reb.has_brake_latch[1] = 1;
        require(bike::physics_core::rebase_world_rotations_2pi<double, CpuExactToolkit>(&reb, err64) == Status::ok,
                "rebase_world_rotations_2pi");
        require(std::abs(reb.front_wheel_qpos) <= std::numbers::pi + 1e-12 &&
                    std::abs(reb.rear_wheel_qpos) <= std::numbers::pi + 1e-12 &&
                    std::abs(reb.crank_qpos) <= std::numbers::pi + 1e-12 &&
                    std::abs(reb.drive_shaft_qpos) <= std::numbers::pi + 1e-12,
                "rebased qpos in [-pi, pi]");
        require(std::abs((reb.front_wheel_qpos - reb.brake_latch[0]) - 0.001) <= 1e-11,
                "rebased brake latch invariant");

        const float qw0 = static_cast<float>(reb.rear_wheel_qpos);
        const float qs0 = static_cast<float>(reb.drive_shaft_qpos);
        const float qc0 = static_cast<float>(reb.crank_qpos);
        const float qw1 = static_cast<float>(reb.rear_wheel_qpos + 0.05);
        const float qs1 = static_cast<float>(reb.drive_shaft_qpos + 0.05 / r64 + 0.000136);
        const float qc1 = static_cast<float>(reb.crank_qpos + 0.05 / r64 + 0.0002);

        bool hr_fh = false, hr_cl = false;
        float br_fh = 0.0f, br_cl = 0.0f;
        (void)bike::physics_core::ratchet_reset_bound<float, GpuToolkit>(
            r32, qs0, qw0, hr_fh, br_fh, err32);
        (void)bike::physics_core::ratchet_reset_bound<float, GpuToolkit>(
            1.0f, qc0, qs0, hr_cl, br_cl, err32);
        (void)bike::physics_core::ratchet_prepare_bound<float, GpuToolkit>(
            r32, qs1, qw1, hr_fh, br_fh, err32);
        (void)bike::physics_core::ratchet_prepare_bound<float, GpuToolkit>(
            1.0f, qc1, qs1, hr_cl, br_cl, err32);
        const float gap_r = (r32 * qs1 - qw1) - br_fh;
        const float cgap_r = (qc1 - qs1) - br_cl;
        max_fh_reb = std::max(max_fh_reb, std::abs(static_cast<double>(gap_r) - gap64));
        max_cl_reb = std::max(max_cl_reb, std::abs(static_cast<double>(cgap_r) - cgap64));
    }
    require(max_fh_raw > 1.0e-4, "freehub raw q=5000 cancellation observed");
    require(max_cl_raw > 1.0e-4, "clutch raw q=5000 cancellation observed");
    require(max_fh_reb <= 5.0e-6, "freehub rebased q=5000 FP32 <= 5e-6 rad");
    require(max_cl_reb <= 5.0e-6, "clutch rebased q=5000 FP32 <= 5e-6 rad");
}

#if defined(__APPLE__)
[[gnu::noinline]] void test_metal_parity(const mjModel *m, mjData *d, std::mt19937_64 &rng) {
    namespace tm = bike::physics_core::test_sp2_metal;
    auto req_ptr = std::make_unique<tm::MetalSp2Request>();
    auto &req = *req_ptr;
    req.physics_core_dir = BIKE_PHYSICS_CORE_SRC_DIR;

    // 1. logf & powf (256 points each)
    for (int i = 0; i < 256; ++i) {
        const float x = 0.01f + static_cast<float>(i) * 0.5f;
        req.log_in.push_back(x);
        req.pow_base.push_back(x);
        req.pow_exp.push_back(0.10f + static_cast<float>(i % 15) * 0.1f);
    }

    // 2. Suspension (256 cases: 128 open + 128 lockout_firm)
    const auto cfg_base = make_stock_suspension_cfg();
    auto cfg_lock = cfg_base;
    cfg_lock.shock_damper.lockout_firm = true;
    const auto p32_base = to_core_susp<float>(cfg_base);
    const auto p32_lock = to_core_susp<float>(cfg_lock);
    std::uniform_real_distribution<float> d_fq(-0.004f, 0.178f);
    std::uniform_real_distribution<float> d_sq(-0.006f, 0.072f);
    std::uniform_real_distribution<float> d_v(-2.8f, 2.8f);
    for (int i = 0; i < 256; ++i) {
        req.susp_params.push_back(i < 128 ? p32_base : p32_lock);
        req.susp_fork_q.push_back(d_fq(rng));
        req.susp_fork_v.push_back(d_v(rng));
        req.susp_shock_q.push_back(d_sq(rng));
        req.susp_shock_v.push_back(d_v(rng));
    }

    // 3. Resistance on welded.xml
    engine::reset_data(m, d);
    {
        const auto qpos_mut = std::views::counted(d->qpos, m->nq);
        const auto qvel_mut = std::views::counted(d->qvel, m->nv);
        qpos_mut[2] = 0.18;
        qvel_mut[0] = 6.5;
        qvel_mut[2] = 0.4;
    }
    engine::forward(m, d);

    const int frame_id = mj_name2id(m, mjOBJ_BODY, "frame");
    const int front_id = mj_name2id(m, mjOBJ_BODY, "front_wheel");
    const int rear_id = mj_name2id(m, mjOBJ_BODY, "rear_wheel");
    req.res_nv = static_cast<i32>(m->nv);
    req.res_njnt = static_cast<i32>(m->njnt);
    req.res_cfg = {
        .crr = 0.015f,
        .rolling_taper_rad_s = 0.2f,
        .rho_kg_m3 = 1.2f,
        .cda_m2 = 0.5f,
        .wind_world_mps = {0.5f, 0.0f, 0.0f},
        .point_body_m = {0.0f, 0.0f, 0.5f},
    };
    const auto req_wc = std::span(req.res_wheel_chains);
    req_wc[0] = bike::physics_core::build_wheel_chain(m, front_id);
    req_wc[1] = bike::physics_core::build_wheel_chain(m, rear_id);
    req.res_frame_chain = bike::physics_core::build_wheel_chain(m, frame_id);

    const auto xpos = model_access::readonly_buffer(d->xpos, 3 * m->nbody);
    const auto xmat = model_access::readonly_buffer(d->xmat, 9 * m->nbody);
    const auto qvel = model_access::readonly_buffer(d->qvel, m->nv);
    const auto xanchor = model_access::readonly_buffer(d->xanchor, 3 * m->njnt);
    const auto xaxis = model_access::readonly_buffer(d->xaxis, 3 * m->njnt);

    for (const mjtNum v : xpos) req.res_body_xpos.push_back(static_cast<float>(v));
    for (const mjtNum v : xanchor) req.res_jnt_xanchor.push_back(static_cast<float>(v));
    for (const mjtNum v : xaxis) req.res_jnt_xaxis.push_back(static_cast<float>(v));
    for (const mjtNum v : qvel) req.res_qvel.push_back(static_cast<float>(v));
    for (std::size_t k = 0; k < 3u; ++k) {
        req.res_front_xpos.push_back(static_cast<float>(xpos[static_cast<std::size_t>(3 * front_id) + k]));
        req.res_rear_xpos.push_back(static_cast<float>(xpos[static_cast<std::size_t>(3 * rear_id) + k]));
        req.res_frame_xpos.push_back(static_cast<float>(xpos[static_cast<std::size_t>(3 * frame_id) + k]));
    }
    for (std::size_t k = 0; k < 9u; ++k) {
        req.res_front_xmat.push_back(static_cast<float>(xmat[static_cast<std::size_t>(9 * front_id) + k]));
        req.res_rear_xmat.push_back(static_cast<float>(xmat[static_cast<std::size_t>(9 * rear_id) + k]));
        req.res_frame_xmat.push_back(static_cast<float>(xmat[static_cast<std::size_t>(9 * frame_id) + k]));
    }
    for (std::size_t p = 0; p < 128u; ++p) {
        req.res_front_loads.push_back(5.0f + static_cast<float>(p) * 0.1f);
        req.res_rear_loads.push_back(6.0f + static_cast<float>(p) * 0.1f);
        req.res_front_work.push_back((p % 7u != 0u) ? 1u : 0u);
        req.res_rear_work.push_back((p % 5u != 0u) ? 1u : 0u);
    }
    req.res_front_eff_r = 0.365f;
    req.res_rear_eff_r = 0.345f;

    // 4. Brake hold
    req.brake_spec = {
        .torque_nm = 200.0f,
        .stiffness_nm_rad = 20000.0f,
        .damping_nms_rad = 10.0f,
        .engage_radps = 0.5f,
        .release_radps = 1.0f,
    };
    req.brake_ceiling_nm = 200.0f;
    req.brake_front_demand = 0.9f;
    req.brake_rear_demand = 0.7f;
    req.brake_theta_init = 25.134f;
    req.brake_omega = -0.15f;
    req.brake_latch_init = 25.132f;

    // 5. Envelope
    const auto limited = model_access::readonly_buffer(m->jnt_limited, m->njnt);
    const auto range = model_access::readonly_buffer(m->jnt_range, 2 * m->njnt);
    const auto qp_adr = model_access::readonly_buffer(m->jnt_qposadr, m->njnt);
    const auto dof_adr = model_access::readonly_buffer(m->jnt_dofadr, m->njnt);
    req.env_nq = static_cast<i32>(m->nq);
    req.env_nv = static_cast<i32>(m->nv);
    req.env_qpos.assign(static_cast<std::size_t>(m->nq), 0.0f);
    const auto env_qp = std::span(req.env_spec.qpos_adrs);
    const auto env_dof = std::span(req.env_spec.dof_adrs);
    const auto env_lo = std::span(req.env_spec.lower);
    const auto env_hi = std::span(req.env_spec.upper);
    for (int j = 0; j < m->njnt; ++j) {
        if (limited[static_cast<std::size_t>(j)] == 0) continue;
        const char *name = mj_id2name(m, mjOBJ_JOINT, j);
        if (name == nullptr || !std::string_view(name).starts_with("rider_")) continue;
        const std::size_t idx = static_cast<std::size_t>(req.env_spec.count++);
        env_qp[idx] = qp_adr[static_cast<std::size_t>(j)];
        env_dof[idx] = dof_adr[static_cast<std::size_t>(j)];
        env_lo[idx] = static_cast<float>(range[2u * static_cast<std::size_t>(j)]);
        env_hi[idx] = static_cast<float>(range[2u * static_cast<std::size_t>(j) + 1u]);
        req.env_qpos[static_cast<std::size_t>(env_qp[idx])] = env_lo[idx] + 0.02f;
    }

    // 6. Ratchet + rebase
    req.rebase_in = {
        .front_wheel_qpos = 19.0f,
        .rear_wheel_qpos = 18.9f,
        .crank_qpos = 44.0f,
        .drive_shaft_qpos = 43.9f,
        .freehub_ratio = 22.0f / 51.0f,
        .freehub_boundary = -0.1f,
        .clutch_boundary = 0.05f,
        .brake_latch = {18.999f, 18.898f},
        .has_freehub_boundary = 1,
        .has_clutch_boundary = 1,
        .has_brake_latch = {1, 1},
    };
    req.ratchet_old_ratio = 22.0f / 51.0f;
    req.ratchet_new_ratio = 22.0f / 39.0f;
    req.active_cog = 9;

    const auto gpu_ptr = std::make_unique<tm::MetalSp2Output>(tm::run_metal_sp2_parity(req));
    const auto &gpu = *gpu_ptr;
    require(gpu.device_available, "Metal device available");
    if (!gpu.error_message.empty()) {
        throw std::runtime_error("Metal SP2 failure: " + gpu.error_message);
    }

    // Assert 0-bit Host float vs Metal MTLMathModeSafe parity on ALL outputs
    for (std::size_t i = 0; i < req.log_in.size(); ++i) {
        require_bits_f32(gpu.log_out[i], GpuToolkit::log(req.log_in[i]), "Metal logf_portable 0-bit");
        require_bits_f32(gpu.pow_out[i], GpuToolkit::pow_py(req.pow_base[i], req.pow_exp[i]),
                         "Metal powf_portable 0-bit");
    }

    for (std::size_t i = 0; i < req.susp_params.size(); ++i) {
        SuspensionResult<float> host_r{};
        ErrCode err = ErrCode::none;
        const Status st = bike::physics_core::eval_suspension<float, GpuToolkit>(
            &req.susp_params[i], req.susp_fork_q[i], req.susp_fork_v[i],
            req.susp_shock_q[i], req.susp_shock_v[i], host_r, err);
        require(gpu.susp_status[i] == static_cast<u8>(st), "Metal susp status 0-bit");
        const auto &mr = gpu.susp_out[i];
        require_bits_f32(mr.fork_spring, host_r.fork_spring, "Metal fork_spring 0-bit");
        require_bits_f32(mr.fork_damper, host_r.fork_damper, "Metal fork_damper 0-bit");
        require_bits_f32(mr.shock_spring, host_r.shock_spring, "Metal shock_spring 0-bit");
        require_bits_f32(mr.shock_bumper, host_r.shock_bumper, "Metal shock_bumper 0-bit");
        require_bits_f32(mr.shock_damper, host_r.shock_damper, "Metal shock_damper 0-bit");
        require_bits_f32(mr.shock_top_out, host_r.shock_top_out, "Metal shock_top_out 0-bit");
        require_bits_f32(mr.shock_upper_stop, host_r.shock_upper_stop, "Metal shock_upper_stop 0-bit");
        require_bits_f32(mr.shock_hbo, host_r.shock_hbo, "Metal shock_hbo 0-bit");
        require_bits_f32(mr.shock_total, host_r.shock_total, "Metal shock_total 0-bit");
    }

    {
        const std::size_t nv = static_cast<std::size_t>(req.res_nv);
        std::vector<float> jr(3u * nv), jp(3u * nv), row(nv), roll(nv), aero(nv);
        const ChainWheelJrFn<float, GpuToolkit, const float *, const GpuWheelChain *> wjr{
            .chains = req.res_wheel_chains.data(),
            .jnt_xaxis = req.res_jnt_xaxis.data(),
            .nv = req.res_nv,
            .njnt = req.res_njnt};
        const ChainFrameJpFn<float, GpuToolkit, const float *, const float *, const float *> fjp{
            .chain = req.res_frame_chain,
            .body_xpos = req.res_body_xpos.data(),
            .jnt_xanchor = req.res_jnt_xanchor.data(),
            .jnt_xaxis = req.res_jnt_xaxis.data(),
            .nv = req.res_nv,
            .njnt = req.res_njnt};
        ErrCode err = ErrCode::none;
        ResDiagField diag = ResDiagField::none;
        const Status st = bike::physics_core::eval_resistance<float, GpuToolkit>(
            req.res_nv, &req.res_cfg, req.res_qvel.data(),
            req.res_front_loads.data(), req.res_front_work.data(),
            static_cast<i32>(req.res_front_loads.size()), req.res_front_eff_r,
            req.res_front_xpos.data(), req.res_front_xmat.data(),
            req.res_rear_loads.data(), req.res_rear_work.data(),
            static_cast<i32>(req.res_rear_loads.size()), req.res_rear_eff_r,
            req.res_rear_xpos.data(), req.res_rear_xmat.data(),
            req.res_frame_xpos.data(), req.res_frame_xmat.data(),
            jr.data(), jp.data(), row.data(), roll.data(), aero.data(),
            wjr, fjp, err, diag);
        require(gpu.res_status == static_cast<u8>(st) && st == Status::ok, "Metal res status");
        for (std::size_t i = 0; i < nv; ++i) {
            require_bits_f32(gpu.res_rolling_out[i], roll[i], "Metal resistance rolling 0-bit");
            require_bits_f32(gpu.res_aero_out[i], aero[i], "Metal resistance aero 0-bit");
        }
    }

    {
        std::array<float, 4> fl{};
        bool changed = false;
        ErrCode err = ErrCode::none;
        (void)bike::physics_core::apply_static_brake<float, GpuToolkit>(
            req.brake_ceiling_nm, req.brake_front_demand, req.brake_rear_demand,
            1, 3, 4, fl.data(), changed, err);
        float th = req.brake_theta_init;
        float lt = req.brake_latch_init;
        bool hl = true;
        (void)bike::physics_core::rebase_2pi_angle<float, GpuToolkit>(th, hl, lt);
        float tq = 0.0f;
        (void)bike::physics_core::eval_brake_hold_wheel<float, GpuToolkit>(
            &req.brake_spec, req.brake_front_demand, th, req.brake_omega,
            true, false, hl, lt, tq, err);
        require_bits_f32(gpu.brake_loss_front, fl[1], "Metal brake loss front 0-bit");
        require_bits_f32(gpu.brake_loss_rear, fl[3], "Metal brake loss rear 0-bit");
        require(gpu.brake_changed == (changed ? 1u : 0u), "Metal brake changed");
        require_bits_f32(gpu.brake_hold_torque, tq, "Metal brake hold torque 0-bit");
        require_bits_f32(gpu.brake_rebased_theta, th, "Metal brake rebased theta 0-bit");
        require_bits_f32(gpu.brake_rebased_latch, lt, "Metal brake rebased latch 0-bit");
        require(gpu.brake_has_latch == (hl ? 1u : 0u), "Metal brake has_latch");
    }

    {
        std::vector<float> qfrc(static_cast<std::size_t>(req.env_nv), 0.0f);
        float stored = 0.0f;
        ErrCode err = ErrCode::none;
        const Status st = bike::physics_core::eval_rider_joint_envelope<float, GpuToolkit>(
            &req.env_spec, req.env_nq, req.env_nv, req.env_qpos.data(),
            qfrc.data(), stored, err);
        require(gpu.env_status == static_cast<u8>(st) && st == Status::ok, "Metal env status");
        require_bits_f32(gpu.env_stored_j, stored, "Metal env stored_j 0-bit");
        for (std::size_t i = 0; i < qfrc.size(); ++i) {
            require_bits_f32(gpu.env_qfrc_out[i], qfrc[i], "Metal env qfrc 0-bit");
        }
    }

    {
        WorldRotationsRebaseState<float> st_reb = req.rebase_in;
        ErrCode err = ErrCode::none;
        Status st = bike::physics_core::rebase_world_rotations_2pi<float, GpuToolkit>(&st_reb, err);
        require_bits_f32(gpu.rebase_out.front_wheel_qpos, st_reb.front_wheel_qpos, "Metal rebase fw 0-bit");
        require_bits_f32(gpu.rebase_out.rear_wheel_qpos, st_reb.rear_wheel_qpos, "Metal rebase rw 0-bit");
        require_bits_f32(gpu.rebase_out.crank_qpos, st_reb.crank_qpos, "Metal rebase crank 0-bit");
        require_bits_f32(gpu.rebase_out.drive_shaft_qpos, st_reb.drive_shaft_qpos, "Metal rebase shaft 0-bit");
        require_bits_f32(gpu.rebase_out.freehub_boundary, st_reb.freehub_boundary, "Metal rebase fh_b 0-bit");
        require_bits_f32(gpu.rebase_out.clutch_boundary, st_reb.clutch_boundary, "Metal rebase cl_b 0-bit");
        const auto st_reb_bl = std::span(st_reb.brake_latch);
        const auto gpu_reb_bl = std::span(gpu.rebase_out.brake_latch);
        require_bits_f32(gpu_reb_bl[0], st_reb_bl[0], "Metal rebase br0 0-bit");
        require_bits_f32(gpu_reb_bl[1], st_reb_bl[1], "Metal rebase br1 0-bit");

        bool hb = false;
        float b = 0.0f;
        const float q_drv = st_reb.drive_shaft_qpos;
        const float q_rwn = st_reb.rear_wheel_qpos;
        const float phi = bike::physics_core::ratchet_phi<float, GpuToolkit>(
            req.ratchet_old_ratio, q_drv, q_rwn);
        if (st == Status::ok) {
            st = bike::physics_core::ratchet_reset_bound<float, GpuToolkit>(
                req.ratchet_old_ratio, q_drv, q_rwn, hb, b, err);
        }
        const float b_reset = b;
        if (st == Status::ok) {
            st = bike::physics_core::ratchet_prepare_bound<float, GpuToolkit>(
                req.ratchet_old_ratio, q_drv - 0.001f, q_rwn, hb, b, err);
        }
        const float b_prep = b;
        if (st == Status::ok) {
            st = bike::physics_core::ratchet_shift_ratio_bound<float, GpuToolkit>(
                req.ratchet_old_ratio, req.ratchet_new_ratio, q_drv - 0.001f, q_rwn,
                hb, b, err);
        }
        const float b_shift = b;
        std::array<i32, kCassetteCogCount> tids{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
        std::array<float, 24> ranges{};
        if (st == Status::ok) {
            st = bike::physics_core::fill_cassette_tendon_ranges<float, GpuToolkit>(
                req.active_cog, kCassetteCogCount, kCassetteCogCount, b,
                tids.data(), ranges.data(), err);
        }
        require(gpu.ratchet_status == static_cast<u8>(st) && st == Status::ok, "Metal ratchet status");
        require_bits_f32(gpu.ratchet_phi, phi, "Metal ratchet phi 0-bit");
        require_bits_f32(gpu.ratchet_bound_after_reset, b_reset, "Metal ratchet reset 0-bit");
        require_bits_f32(gpu.ratchet_bound_after_prepare, b_prep, "Metal ratchet prepare 0-bit");
        require_bits_f32(gpu.ratchet_bound_after_shift, b_shift, "Metal ratchet shift 0-bit");
        for (std::size_t i = 0; i < 24u; ++i) {
            require_bits_f32(gpu.cassette_ranges[i], ranges[i], "Metal cassette range 0-bit");
        }
    }

    std::cout << "[test_sp2_force_laws] Apple Metal MTLMathModeSafe 0-bit parity PASS on "
              << gpu.device_name << '\n';
}
#endif

} // namespace

int main() {
    try {
        return bt::run([]() {
            const std::filesystem::path xml_path = bt::fixture("model/welded.xml");
            const ModelPtr m = load_xml_model(xml_path);
            const DataPtr d(engine::make_data(m.get()), delete_data);
            std::seed_seq seed{20261117u};
            std::mt19937_64 rng(seed);

            test_logf_and_powf_portable();
            test_suspension_parity(m.get(), d.get(), rng);
            test_resistance_parity(m.get(), d.get(), rng);
            test_brake_hold_parity(m.get(), d.get(), rng);
            test_joint_envelope_parity(m.get(), rng);
            test_ratchet_parity(xml_path, rng);
#if defined(__APPLE__)
            test_metal_parity(m.get(), d.get(), rng);
#endif
            std::cout << "[test_sp2_force_laws] ALL PASS\n";
        });
    } catch (const std::exception &ex) {
        std::cerr << "[test_sp2_force_laws] FAIL: " << ex.what() << '\n';
        return 1;
    } catch (...) {
        std::cerr << "[test_sp2_force_laws] FAIL: unknown exception\n";
        return 1;
    }
}
