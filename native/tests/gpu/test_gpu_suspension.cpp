// native/tests/gpu/test_gpu_suspension.cpp — Unit and reference-parity tests
// for the single-source physics_core suspension force law wired through batch_step.hpp.
#include "../../src/config_types.hpp"
#include "../../src/gpu/batch_step.hpp"
#include "../../src/model_access.hpp"
#include "../../src/platform/argv.hpp"
#include "../../src/writers/suspension.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <exception>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

namespace fs = std::filesystem;
using namespace bike::gpu;
using bike::physics_core::Status;
using bike::physics_core::SuspensionResult;

using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPtr = std::unique_ptr<mjData, decltype(&mj_deleteData)>;
using TestFunction = void (*)();

struct TestCase {
    std::string_view name;
    TestFunction run;
};

void require(bool condition, std::string_view message) {
    if (!condition) {
        throw std::runtime_error(std::string(message));
    }
}

template <class T>
void require_near(T actual, T expected, T tol, std::string_view what) {
    if (!(std::abs(actual - expected) <= tol)) {
        throw std::runtime_error(std::string(what) +
                                 ": actual=" + std::to_string(actual) +
                                 " expected=" + std::to_string(expected) +
                                 " tol=" + std::to_string(tol));
    }
}

constexpr std::size_t to_index(int field) {
    return static_cast<std::size_t>(field);
}

fs::path find_welded_xml() {
    std::vector<fs::path> candidates;
#ifdef BIKE_REPO_ROOT
    candidates.emplace_back(fs::path(BIKE_REPO_ROOT) / "tests/fixtures/model/welded.xml");
#endif
    candidates.emplace_back("tests/fixtures/model/welded.xml");
    candidates.emplace_back("../../tests/fixtures/model/welded.xml");
    candidates.emplace_back("../../../tests/fixtures/model/welded.xml");
    for (const auto &p : candidates) {
        std::error_code ec;
        if (fs::is_regular_file(p, ec)) {
            return fs::canonical(p, ec);
        }
    }
    return {};
}

nativecfg::SuspensionConfig to_native_cfg(const GpuSuspensionParams &gp) {
    const auto fill_core = [](const bike::physics_core::DamperCoreParams<float> &c,
                              nativecfg::DamperCore &dst) {
        dst.max_hsc = c.max_hsc;
        dst.max_lsc = c.max_lsc;
        dst.max_reb = c.max_reb;
        dst.hsc_clicks = c.hsc_clicks;
        dst.lsc_clicks = c.lsc_clicks;
        dst.rebound_clicks = c.rebound_clicks;
        dst.c_lsc_min = static_cast<double>(c.c_lsc_min);
        dst.c_lsc_max = static_cast<double>(c.c_lsc_max);
        dst.c_hsc_min = static_cast<double>(c.c_hsc_min);
        dst.c_hsc_max = static_cast<double>(c.c_hsc_max);
        dst.c_reb_min = static_cast<double>(c.c_reb_min);
        dst.c_reb_max = static_cast<double>(c.c_reb_max);
        dst.v_knee_comp = static_cast<double>(c.v_knee_comp);
        dst.v_knee_reb = static_cast<double>(c.v_knee_reb);
    };
    const auto &sp = gp.params;
    nativecfg::SuspensionConfig cfg{};
    cfg.physics_mode = "physical";
    cfg.fork_joint = "fork_travel";
    cfg.shock_joint = "shock_stroke";
    cfg.air_spring = {
        .specs = {
            .stanchion_inner_diam_mm = static_cast<double>(sp.air_spring.stanchion_inner_diam_mm),
            .total_travel_mm = static_cast<double>(sp.air_spring.total_travel_mm),
            .pos_chamber_length_mm = static_cast<double>(sp.air_spring.pos_chamber_length_mm),
            .neg_chamber_length_mm = static_cast<double>(sp.air_spring.neg_chamber_length_mm),
            .token_volume_cm3 = static_cast<double>(sp.air_spring.token_volume_cm3),
            .max_tokens = sp.air_spring.max_tokens,
            .gamma = static_cast<double>(sp.air_spring.gamma),
            .atm_pressure_pa = static_cast<double>(sp.air_spring.atm_pressure_pa),
        },
        .num_tokens = sp.air_spring.num_tokens,
        .gauge_pressure_psi = static_cast<double>(sp.air_spring.gauge_pressure_psi),
    };
    fill_core(sp.fork_damper.core, cfg.fork_damper.core);
    cfg.fork_damper.total_travel_mm = static_cast<double>(sp.fork_damper.total_travel_mm);
    cfg.fork_damper.hbo_start_mm = static_cast<double>(sp.fork_damper.hbo_start_mm);
    cfg.fork_damper.c_hbo_base = static_cast<double>(sp.fork_damper.c_hbo_base);
    cfg.fork_damper.legacy_behavior = false;

    fill_core(sp.shock_damper.core, cfg.shock_damper.core);
    cfg.shock_damper.total_stroke_mm = static_cast<double>(sp.shock_damper.total_stroke_mm);
    cfg.shock_damper.max_hbo = sp.shock_damper.max_hbo;
    cfg.shock_damper.hbo_clicks = sp.shock_damper.hbo_clicks;
    cfg.shock_damper.lockout_firm = (sp.shock_damper.lockout_firm != 0);
    cfg.shock_damper.legacy_behavior = false;
    cfg.shock_damper.hbo_start_mm = static_cast<double>(sp.shock_damper.hbo_start_mm);
    cfg.shock_damper.c_hbo_min = static_cast<double>(sp.shock_damper.c_hbo_min);
    cfg.shock_damper.c_hbo_max = static_cast<double>(sp.shock_damper.c_hbo_max);
    cfg.shock_damper.lockout_preload_n = static_cast<double>(sp.shock_damper.lockout_preload_n);
    cfg.shock_damper.lockout_stiffness = static_cast<double>(sp.shock_damper.lockout_stiffness);

    cfg.coil = {
        .rate_n_m = static_cast<double>(sp.coil.rate_n_m),
        .preload_mm = static_cast<double>(sp.coil.preload_mm),
        .stroke_mm = static_cast<double>(sp.coil.stroke_mm),
        .bumper_length_mm = static_cast<double>(sp.coil.bumper_length_mm),
        .bumper_peak_n = static_cast<double>(sp.coil.bumper_peak_n),
        .legacy_behavior = false,
    };
    cfg.end_stops = {
        .stiffness_n_m = static_cast<double>(sp.end_stops.stiffness_n_m),
        .damping_n_s_m = static_cast<double>(sp.end_stops.damping_n_s_m),
    };
    return cfg;
}

void test_default_preloads_and_tokens() {
    const GpuSuspensionParams gp = gpu_make_default_suspension_params();
    require(gp.fork_preload_n > 200.0f && gp.fork_preload_n < 1500.0f,
            "fork sag preload is physically bounded");
    require(gp.shock_preload_n > 1000.0f && gp.shock_preload_n < 4000.0f,
            "shock sag preload is physically bounded");

    GpuSuspensionParams zero_tok = gp;
    zero_tok.params.air_spring.num_tokens = 0;
    GpuSuspensionParams max_tok = gp;
    max_tok.params.air_spring.num_tokens = 4;

    const GpuModelIndices idx{};
    std::array<float, kMaxNq> qpos{};
    std::array<float, kMaxNv> qvel{};
    qpos[to_index(idx.fork_travel_qpos)] = 0.170f;
    std::array<float, kMaxNv> qfrc0{};
    std::array<float, kMaxNv> qfrc4{};
    SuspensionResult<float> res0{};
    SuspensionResult<float> res4{};

    require(gpu_write_suspension_qfrc(zero_tok, idx, qpos.data(), qvel.data(), qfrc0.data(), &res0) == Status::ok,
            "0-token fork evaluation succeeds");
    require(gpu_write_suspension_qfrc(max_tok, idx, qpos.data(), qvel.data(), qfrc4.data(), &res4) == Status::ok,
            "4-token fork evaluation succeeds");
    require(res4.fork_spring > res0.fork_spring * 1.15f,
            "adding air volume tokens increases deep-travel air spring force");
}

void test_parity_with_suspension_writer() {
    const fs::path xml_path = find_welded_xml();
    require(!xml_path.empty(), "located tests/fixtures/model/welded.xml");

    std::array<char, 1024> mj_err{};
    const ModelPtr m{mj_loadXML(xml_path.string().c_str(), nullptr, mj_err.data(),
                                static_cast<int>(mj_err.size())),
                     &mj_deleteModel};
    require(m != nullptr, std::string("mj_loadXML failed: ") + mj_err.data());
    const DataPtr d{mj_makeData(m.get()), &mj_deleteData};
    require(d != nullptr, "mj_makeData succeeds");

    const GpuSuspensionParams gp = gpu_make_default_suspension_params();
    const SuspensionWriter writer(m.get(), to_native_cfg(gp));
    const GpuModelIndices idx{};
    require(writer.fork_qposadr() == idx.fork_travel_qpos, "fork qpos matches GpuModelIndices");
    require(writer.fork_dofadr() == idx.fork_travel_dof, "fork dof matches GpuModelIndices");
    require(writer.shock_qposadr() == idx.shock_stroke_qpos, "shock qpos matches GpuModelIndices");
    require(writer.shock_dofadr() == idx.shock_stroke_dof, "shock dof matches GpuModelIndices");

    struct Sample {
        float fork_q;
        float fork_v;
        float shock_q;
        float shock_v;
    };
    const std::array<Sample, 7> samples{{
        {.fork_q = 0.0f, .fork_v = 0.0f, .shock_q = 0.0f, .shock_v = 0.0f},
        {.fork_q = 0.040f, .fork_v = 0.15f, .shock_q = 0.020f, .shock_v = 0.12f},
        {.fork_q = 0.165f, .fork_v = 1.40f, .shock_q = 0.058f, .shock_v = 0.90f},
        {.fork_q = 0.175f, .fork_v = -1.20f, .shock_q = 0.062f, .shock_v = -0.80f},
        {.fork_q = 0.025f, .fork_v = -0.05f, .shock_q = -0.004f, .shock_v = -0.25f},
        {.fork_q = 0.110f, .fork_v = 0.02f, .shock_q = 0.070f, .shock_v = 0.35f},
        {.fork_q = 0.090f, .fork_v = 0.50f, .shock_q = 0.045f, .shock_v = -0.50f},
    }};

    const std::span<mjtNum> d_qpos = model_access::mutable_buffer(d->qpos, m->nq, "qpos");
    const std::span<mjtNum> d_qvel = model_access::mutable_buffer(d->qvel, m->nv, "qvel");

    for (const Sample &s : samples) {
        std::ranges::fill(d_qpos, 0.0);
        std::ranges::fill(d_qvel, 0.0);
        d_qpos[to_index(idx.fork_travel_qpos)] = static_cast<mjtNum>(s.fork_q);
        d_qvel[to_index(idx.fork_travel_dof)] = static_cast<mjtNum>(s.fork_v);
        d_qpos[to_index(idx.shock_stroke_qpos)] = static_cast<mjtNum>(s.shock_q);
        d_qvel[to_index(idx.shock_stroke_dof)] = static_cast<mjtNum>(s.shock_v);

        const std::vector<SuspensionWriter::Component> comps = writer.components(d.get());
        std::vector<double> ref_qfrc(static_cast<std::size_t>(m->nv), 0.0);
        for (const auto &[name, vec] : comps) {
            (void)name;
            for (int i = 0; i < m->nv; ++i) {
                ref_qfrc[to_index(i)] += vec[to_index(i)];
            }
        }

        std::array<float, kMaxNq> qp32{};
        std::array<float, kMaxNv> qv32{};
        std::array<float, kMaxNv> gpu_qfrc{};
        qp32[to_index(idx.fork_travel_qpos)] = s.fork_q;
        qv32[to_index(idx.fork_travel_dof)] = s.fork_v;
        qp32[to_index(idx.shock_stroke_qpos)] = s.shock_q;
        qv32[to_index(idx.shock_stroke_dof)] = s.shock_v;

        require(gpu_write_suspension_qfrc(gp, idx, qp32.data(), qv32.data(), gpu_qfrc.data()) == Status::ok,
                "gpu_write_suspension_qfrc parity sample ok");

        for (const int dof : {idx.fork_travel_dof, idx.shock_stroke_dof}) {
            const float expected = static_cast<float>(ref_qfrc[to_index(dof)]);
            const float tol = 2e-4f * std::max(1.0f, std::abs(expected));
            require_near(gpu_qfrc[to_index(dof)], expected, tol,
                         "GPU suspension matches FP64 SuspensionWriter");
        }
    }
}

void test_hbo_bumper_endstops_and_guards() {
    const GpuSuspensionParams gp = gpu_make_default_suspension_params();
    const GpuModelIndices idx{};

    // HBO engagement above hbo_start_mm = 160 mm when compressing
    std::array<float, kMaxNq> qp{};
    std::array<float, kMaxNv> qv{};
    qv[to_index(idx.fork_travel_dof)] = 1.0f;
    qp[to_index(idx.fork_travel_qpos)] = 0.140f;
    SuspensionResult<float> r_pre_hbo{};
    std::array<float, kMaxNv> qf_dummy{};
    require(gpu_write_suspension_qfrc(gp, idx, qp.data(), qv.data(), qf_dummy.data(), &r_pre_hbo) == Status::ok,
            "pre-HBO evaluation ok");
    qp[to_index(idx.fork_travel_qpos)] = 0.175f;
    SuspensionResult<float> r_in_hbo{};
    require(gpu_write_suspension_qfrc(gp, idx, qp.data(), qv.data(), qf_dummy.data(), &r_in_hbo) == Status::ok,
            "in-HBO evaluation ok");
    require(r_in_hbo.fork_damper > r_pre_hbo.fork_damper + 500.0f,
            "Charger3 HBO ramps compression damping past hbo_start_mm");

    // Top-out restoration when shock_qpos < 0
    std::ranges::fill(qp, 0.0f);
    std::ranges::fill(qv, 0.0f);
    qp[to_index(idx.shock_stroke_qpos)] = -0.005f;
    qv[to_index(idx.shock_stroke_dof)] = -0.1f;
    std::array<float, kMaxNv> qf_top{};
    SuspensionResult<float> r_top{};
    require(gpu_write_suspension_qfrc(gp, idx, qp.data(), qv.data(), qf_top.data(), &r_top) == Status::ok,
            "top-out evaluation ok");
    require(r_top.shock_top_out < -1000.0f && qf_top[to_index(idx.shock_stroke_dof)] > 1000.0f,
            "shock top-out pulls stroke back toward zero");

    // Pre-populated qfrc accumulates rather than overwrites, and disabled DOFs are ignored
    GpuModelIndices disabled_idx = idx;
    disabled_idx.fork_travel_dof = -1;
    disabled_idx.shock_stroke_dof = -1;
    std::array<float, kMaxNv> qf_acc{};
    qf_acc[to_index(idx.fork_travel_dof)] = 42.5f;
    require(gpu_write_suspension_qfrc(gp, disabled_idx, qp.data(), qv.data(), qf_acc.data()) == Status::ok,
            "disabled DOF evaluation ok");
    require_near(qf_acc[to_index(idx.fork_travel_dof)], 42.5f, 1e-6f,
                 "disabled fork DOF leaves existing qfrc intact");

    // Non-finite velocity triggers overflow without mutating qfrc
    qv[to_index(idx.fork_travel_dof)] = std::numeric_limits<float>::infinity();
    require(gpu_write_suspension_qfrc(gp, idx, qp.data(), qv.data(), qf_acc.data()) == Status::overflow,
            "non-finite fork velocity returns Status::overflow");
    require_near(qf_acc[to_index(idx.fork_travel_dof)], 42.5f, 1e-6f,
                 "failed evaluation does not mutate qfrc");
}

constexpr std::array<TestCase, 3> cases{{
    {.name = "default_preloads_and_tokens", .run = test_default_preloads_and_tokens},
    {.name = "parity_with_suspension_writer", .run = test_parity_with_suspension_writer},
    {.name = "hbo_bumper_endstops_and_guards", .run = test_hbo_bumper_endstops_and_guards},
}};

int run_case(const TestCase &test_case) {
    try {
        test_case.run();
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
            for (const TestCase &test_case : cases) {
                std::cout << test_case.name << '\n';
            }
            return 0;
        }
        if (arguments.size() > 2) {
            std::cerr << "usage: bike_test_gpu_suspension [case-name|--list]\n";
            return 2;
        }
        const std::string_view requested =
            arguments.size() == 1 ? "all" : std::string_view(arguments[1]);
        bool found = requested == "all";
        int status = 0;
        for (const TestCase &test_case : cases) {
            if (requested == "all" || requested == test_case.name) {
                found = true;
                if (run_case(test_case) != 0) {
                    status = 1;
                }
            }
        }
        if (!found) {
            std::cerr << "unknown gpu suspension case: " << requested << '\n';
            return 2;
        }
        return status;
    } catch (const std::exception &error) {
        std::cerr << "FAIL argument handling: " << error.what() << '\n';
        return 1;
    }
}
