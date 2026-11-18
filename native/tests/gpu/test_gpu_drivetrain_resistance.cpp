// native/tests/gpu/test_gpu_drivetrain_resistance.cpp — Unit and reference-parity
// tests for the single-source physics_core resistance, ratchet, and brake_hold laws
// wired through native/src/gpu/batch_step.hpp.
#include "../../src/config_types.hpp"
#include "../../src/gpu/batch_step.hpp"
#include "../../src/gpu/model_assets.hpp"
#include "../../src/model_access.hpp"
#include "../../src/physics_core/cpu_exact_toolkit.hpp"
#include "../../src/physics_core/tyre_upload.hpp"
#include "../../src/platform/argv.hpp"
#include "../../src/writers/resistance.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

namespace fs = std::filesystem;
using namespace bike::gpu;
using bike::physics_core::HostTyreUploadBundle;
using bike::physics_core::Status;

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

nativecfg::TireConfig make_compliant_cfg() {
    nativecfg::TireConfig cfg{};
    cfg.backend = "compliant_2d";
    cfg.significant_delta_m = 1e-4;
    cfg.significance_fraction = 0.05;
    cfg.distinct_normal_deg = 12.0;
    cfg.surface_mode = "configured";
    nativecfg::TireMaterial mat{};
    mat.radial_k_n_m = 110000.0;
    mat.loss_factor = 0.05;
    mat.hysteresis_rate_eps_mps = 0.01;
    mat.valid_load_range_n = {0.0, 5000.0};
    cfg.front.material = mat;
    cfg.front.tangent_k_n_m = 60000.0;
    cfg.front.mu = 1.1;
    cfg.front.relaxation_length_m = 0.05;
    cfg.rear.material = mat;
    cfg.rear.tangent_k_n_m = 60000.0;
    cfg.rear.mu = 1.1;
    cfg.rear.relaxation_length_m = 0.05;
    return cfg;
}

ModelPtr load_welded_model(const fs::path &xml_path) {
    std::string mj_err(1024, '\0');
    mjModel *raw = mj_loadXML(xml_path.string().c_str(), nullptr, mj_err.data(),
                              static_cast<int>(mj_err.size()));
    require(raw != nullptr, "mj_loadXML failed: " + mj_err);
    return {raw, &mj_deleteModel};
}

void test_resistance_parity_with_writer() {
    const fs::path xml_path = find_welded_xml();
    require(!xml_path.empty(), "located tests/fixtures/model/welded.xml");

    const ModelPtr m = load_welded_model(xml_path);
    const DataPtr d{mj_makeData(m.get()), &mj_deleteData};
    require(d != nullptr, "mj_makeData succeeds");
    mj_forward(m.get(), d.get());

    const GpuModelIndices idx = assets::resolve_gpu_model_indices(m.get());
    const int front_geom = mj_name2id(m.get(), mjOBJ_GEOM, "geom_front_contact");
    const int rear_geom = mj_name2id(m.get(), mjOBJ_GEOM, "geom_rear_contact");
    require(front_geom >= 0 && rear_geom >= 0, "welded wheel contact geoms resolved");

    const auto bundle = std::make_unique<HostTyreUploadBundle>(
        bike::physics_core::build_gpu_tyre_model(
            m.get(), d.get(), make_compliant_cfg(),
            front_geom, rear_geom, idx.front_wheel_body, idx.rear_wheel_body, idx.frame_body));

    nativecfg::ResistanceConfig rcfg{};
    rcfg.crr = 0.012;
    rcfg.rolling_taper_rad_s = 0.25;
    rcfg.rho_kg_m3 = 1.225;
    rcfg.cda_m2 = 0.42;
    rcfg.wind_world_mps = {0.8, 0.0, -0.2};
    rcfg.point_body_m = {0.0, 0.0, 0.45};
    rcfg.frame_body = "frame";
    rcfg.front_wheel_body = "front_wheel";
    rcfg.rear_wheel_body = "rear_wheel";

    const ResistanceWriter rw(m.get(), rcfg);

    bike::physics_core::ResistanceParams<float> rp32{};
    rp32.crr = static_cast<float>(rcfg.crr);
    rp32.rolling_taper_rad_s = static_cast<float>(rcfg.rolling_taper_rad_s);
    rp32.rho_kg_m3 = static_cast<float>(rcfg.rho_kg_m3);
    rp32.cda_m2 = static_cast<float>(rcfg.cda_m2);
    rp32.wind_world_mps[0] = static_cast<float>(rcfg.wind_world_mps[0]);
    rp32.wind_world_mps[2] = static_cast<float>(rcfg.wind_world_mps[2]);
    rp32.point_body_m[2] = static_cast<float>(rcfg.point_body_m[2]);

    const std::span<mjtNum> d_qpos = model_access::mutable_buffer(d->qpos, m->nq, "qpos");
    const std::span<mjtNum> d_qvel = model_access::mutable_buffer(d->qvel, m->nv, "qvel");
    d_qpos[to_index(idx.root_pitch_qpos)] = 0.14;
    d_qpos[to_index(idx.fork_travel_qpos)] = 0.05;
    d_qvel[to_index(idx.root_x_dof)] = 7.5;
    d_qvel[to_index(idx.root_z_dof)] = -0.6;
    d_qvel[to_index(idx.root_pitch_dof)] = 0.35;
    d_qvel[to_index(idx.front_wheel_spin_dof)] = 20.5;
    d_qvel[to_index(idx.rear_wheel_spin_dof)] = 21.3;
    d_qvel[to_index(idx.main_pivot_dof)] = 0.4;
    d_qvel[to_index(idx.horst_pivot_dof)] = -0.25;
    mj_forward(m.get(), d.get());

    const std::array<double, 1> f_loads64{450.0};
    const std::array<double, 1> r_loads64{580.0};
    const std::array<bool, 1> work64{true};
    const double f_eff = 0.362;
    const double r_eff = 0.344;
    std::array<ForceComponentView, ResistanceWriter::kComponentCount> views{};
    rw.components_into(
        d.get(),
        TireSideInput{.patch_loads = f_loads64, .patch_working = work64, .effective_radius_m = f_eff},
        TireSideInput{.patch_loads = r_loads64, .patch_working = work64, .effective_radius_m = r_eff},
        views);

    std::vector<float> qvel32(static_cast<std::size_t>(m->nv));
    for (int i = 0; i < m->nv; ++i) {
        qvel32[to_index(i)] = static_cast<float>(d_qvel[to_index(i)]);
    }
    const std::span<const mjtNum> src_xpos = model_access::readonly_buffer(d->xpos, 3 * m->nbody, "xpos");
    const std::span<const mjtNum> src_xmat = model_access::readonly_buffer(d->xmat, 9 * m->nbody, "xmat");
    const std::span<const mjtNum> src_xanc = model_access::readonly_buffer(d->xanchor, 3 * m->njnt, "xanchor");
    const std::span<const mjtNum> src_xax = model_access::readonly_buffer(d->xaxis, 3 * m->njnt, "xaxis");

    std::vector<float> xpos32(src_xpos.size());
    std::vector<float> xmat32(src_xmat.size());
    std::vector<float> xanc32(src_xanc.size());
    std::vector<float> xax32(src_xax.size());
    for (std::size_t i = 0; i < src_xpos.size(); ++i) xpos32[i] = static_cast<float>(src_xpos[i]);
    for (std::size_t i = 0; i < src_xmat.size(); ++i) xmat32[i] = static_cast<float>(src_xmat[i]);
    for (std::size_t i = 0; i < src_xanc.size(); ++i) xanc32[i] = static_cast<float>(src_xanc[i]);
    for (std::size_t i = 0; i < src_xax.size(); ++i) xax32[i] = static_cast<float>(src_xax[i]);

    std::vector<float> qfrc32(static_cast<std::size_t>(m->nv), 0.0f);
    require(gpu_write_resistance_qfrc(
                rp32, &bundle->gpu_model, idx, qvel32.data(),
                xpos32.data(), xmat32.data(), xanc32.data(), xax32.data(),
                static_cast<float>(f_loads64[0]), static_cast<float>(r_loads64[0]),
                static_cast<float>(f_eff), static_cast<float>(r_eff),
                qfrc32.data()) == Status::ok,
            "gpu_write_resistance_qfrc succeeds");

    float max_abs_ref = 0.0f;
    for (int i = 0; i < m->nv; ++i) {
        const double ref = views[0].values[to_index(i)] + views[1].values[to_index(i)];
        max_abs_ref = std::max(max_abs_ref, static_cast<float>(std::abs(ref)));
        const float tol = 5e-5f * std::max(1.0f, static_cast<float>(std::abs(ref)));
        require_near(qfrc32[to_index(i)], static_cast<float>(ref), tol,
                     "GPU resistance matches FP64 ResistanceWriter");
    }
    require(max_abs_ref > 5.0f, "resistance torque/drag is meaningfully non-zero");
}

void test_ratchet_dual_and_freehub_parity() {
    const GpuDrivetrainParams dp{};
    require_near(gpu_drivetrain_ratio(dp), 22.0f / 51.0f, 1e-6f, "default ratio 22/51");

    const GpuModelIndices idx_27{};
    GpuModelIndices idx_26 = idx_27;
    idx_26.nq = 26;
    idx_26.nv = 26;
    idx_26.drive_shaft_spin_qpos = -1;
    idx_26.drive_shaft_spin_dof = -1;
    idx_26.crank_clutch_tendon = -1;

    bool ref_has = false;
    double ref_bound = 0.0;
    bike::physics_core::ErrCode ref_err = bike::physics_core::ErrCode::none;
    require(bike::physics_core::ratchet_reset_bound<double, bike::physics_core::CpuExactToolkit>(
                22.0 / 51.0, 0.5, 3.0, ref_has, ref_bound, ref_err) == Status::ok,
            "FP64 ratchet reset ok");

    GpuDrivetrainState st_26{};
    require(gpu_reset_drivetrain_state(dp, st_26, 0.5f, 3.0f, 0.0f, false) == Status::ok,
            "nq=26 reset ok");
    require_near(st_26.freehub.boundary, static_cast<float>(ref_bound), 1e-6f,
                 "reset boundary matches FP64 ratchet_reset_bound");

    // Coasting then engaging steps
    std::array<float, kMaxNq> qp{};
    std::array<float, kMaxNv> qv{};
    std::array<float, kMaxNv> qf_tendon{};
    std::array<float, kMaxNv> qf_explicit{};
    GpuDrivetrainOutput out{};

    qp[to_index(idx_26.crank_spin_qpos)] = 0.6f;
    qp[to_index(idx_26.rear_wheel_spin_qpos)] = 4.2f;
    require(bike::physics_core::ratchet_prepare_bound<double, bike::physics_core::CpuExactToolkit>(
                22.0 / 51.0, 0.6, 4.2, ref_has, ref_bound, ref_err) == Status::ok,
            "FP64 ratchet prepare ok");
    require(gpu_write_drivetrain_qfrc(dp, idx_26, st_26, qp.data(), qv.data(),
                                      0.0f, 0.0f, 0.0f, qf_tendon.data(), out, false) == Status::ok,
            "coasting ratchet prepare ok");
    require_near(out.tendon_upper_bound_rad, static_cast<float>(ref_bound), 1e-6f,
                 "coasting boundary matches FP64 ratchet_prepare_bound");
    require(out.freehub_engaged == 0, "coasting freehub not engaged");

    // Dual ratchet with nq=27 drive_shaft_spin
    GpuDrivetrainState st_27{};
    require(gpu_reset_drivetrain_state(dp, st_27, 2.0f, 1.0f, 1.5f, true) == Status::ok,
            "nq=27 dual ratchet reset ok");
    require_near(st_27.freehub.boundary, (22.0f / 51.0f) * 1.5f - 1.0f, 1e-6f,
                 "freehub uses drive_shaft as driver");
    require_near(st_27.crank_clutch.boundary, 2.0f - 1.5f, 1e-6f,
                 "crank_clutch uses crank - drive_shaft");

    // Crank advances beyond boundary while drive shaft lags -> clutch engages
    qp[to_index(idx_27.crank_spin_qpos)] = 2.4f;
    qp[to_index(idx_27.drive_shaft_spin_qpos)] = 1.6f;
    qp[to_index(idx_27.rear_wheel_spin_qpos)] = 0.9f;
    qv[to_index(idx_27.crank_spin_dof)] = 5.0f;
    std::ranges::fill(qf_tendon, 0.0f);
    require(gpu_write_drivetrain_qfrc(dp, idx_27, st_27, qp.data(), qv.data(),
                                      0.8f, 0.0f, 0.0f, qf_tendon.data(), out, false) == Status::ok,
            "nq=27 tendon mode write ok");
    require(out.crank_clutch_engaged == 1 && out.freehub_engaged == 1,
            "both crank_clutch and freehub detect engagement");

    // Standalone rear-drive mode vs tendon mode
    std::ranges::fill(qf_explicit, 0.0f);
    GpuDrivetrainState st_exp{};
    require(gpu_reset_drivetrain_state(dp, st_exp, 2.0f, 1.0f, 1.5f, true) == Status::ok,
            "explicit reset ok");
    GpuDrivetrainOutput out_exp{};
    require(gpu_write_drivetrain_qfrc(dp, idx_27, st_exp, qp.data(), qv.data(),
                                      0.8f, 0.0f, 0.0f, qf_explicit.data(), out_exp, true) == Status::ok,
            "explicit mode write ok");
    require(qf_explicit[to_index(idx_27.rear_wheel_spin_dof)] > 1.0f &&
                qf_tendon[to_index(idx_27.rear_wheel_spin_dof)] == 0.0f,
            "explicit mode applies pawl drive force while tendon mode omits it");
    require(qf_explicit[to_index(idx_27.drive_shaft_spin_dof)] < 0.0f,
            "explicit mode applies reaction torque to drive_shaft_spin");
}

void test_brake_hold_and_dynamic_braking() {
    const GpuDrivetrainParams dp{};
    const GpuModelIndices idx{};
    GpuDrivetrainState st{};
    require(gpu_reset_drivetrain_state(dp, st, 0.0f, 0.0f, 0.0f, true) == Status::ok, "reset ok");

    std::array<float, kMaxNq> qp{};
    std::array<float, kMaxNv> qv{};
    std::array<float, kMaxNv> qf{};
    GpuDrivetrainOutput out{};

    // 1. High and intermediate rolling speed -> does not latch, applies smooth tanhf_portable dynamic braking torque
    qv[to_index(idx.front_wheel_spin_dof)] = 10.0f;
    qv[to_index(idx.rear_wheel_spin_dof)] = 10.0f;
    require(gpu_write_drivetrain_qfrc(dp, idx, st, qp.data(), qv.data(),
                                      0.0f, 0.5f, 1.0f, qf.data(), out, false) == Status::ok,
            "rolling brake ok");
    require(out.front_brake_latched == 0 && out.rear_brake_latched == 0,
            "high speed wheels do not latch");
    require_near(out.front_brake_torque_nm, -210.0f, 1e-4f, "front rolling dynamic brake");
    require_near(out.rear_brake_torque_nm, -380.0f, 1e-4f, "rear rolling dynamic brake");

    // Intermediate rolling speed above engage_radps (0.5) exercises smooth tanhf_portable(omega / v_eps)
    qv[to_index(idx.front_wheel_spin_dof)] = 0.75f;
    qv[to_index(idx.rear_wheel_spin_dof)] = -0.75f;
    std::ranges::fill(qf, 0.0f);
    require(gpu_write_drivetrain_qfrc(dp, idx, st, qp.data(), qv.data(),
                                      0.0f, 0.5f, 1.0f, qf.data(), out, false) == Status::ok,
            "intermediate rolling brake ok");
    require(out.front_brake_latched == 0 && out.rear_brake_latched == 0,
            "intermediate speed wheels above engage_radps do not latch");
    require_near(out.front_brake_torque_nm,
                 -0.5f * dp.front_max_brake_torque_nm * bike::physics_core::tanhf_portable(1.5f),
                 1e-4f, "front intermediate smooth tanhf_portable rolling brake");
    require_near(out.rear_brake_torque_nm,
                 -1.0f * dp.rear_max_brake_torque_nm * bike::physics_core::tanhf_portable(-1.5f),
                 1e-4f, "rear intermediate smooth tanhf_portable rolling brake");

    // 1b. Braking cutout gate: pedaling demand is cut to zero whenever front or rear brake is active
    qv[to_index(idx.crank_spin_dof)] = 5.0f;
    qv[to_index(idx.front_wheel_spin_dof)] = 10.0f;
    qv[to_index(idx.rear_wheel_spin_dof)] = 10.0f;
    std::ranges::fill(qf, 0.0f);
    require(gpu_write_drivetrain_qfrc(dp, idx, st, qp.data(), qv.data(),
                                      0.8f, 0.25f, 0.0f, qf.data(), out, false) == Status::ok,
            "front brake cutout ok");
    require_near(out.crank_torque_nm, 0.0f, 1e-6f, "front brake cuts crank pedaling torque to zero");
    require_near(qf[to_index(idx.crank_spin_dof)], 0.0f, 1e-6f, "front brake writes zero crank qfrc");

    std::ranges::fill(qf, 0.0f);
    require(gpu_write_drivetrain_qfrc(dp, idx, st, qp.data(), qv.data(),
                                      0.8f, 0.0f, 0.25f, qf.data(), out, false) == Status::ok,
            "rear brake cutout ok");
    require_near(out.crank_torque_nm, 0.0f, 1e-6f, "rear brake cuts crank pedaling torque to zero");
    require_near(qf[to_index(idx.crank_spin_dof)], 0.0f, 1e-6f, "rear brake writes zero crank qfrc");

    std::ranges::fill(qf, 0.0f);
    require(gpu_write_drivetrain_qfrc(dp, idx, st, qp.data(), qv.data(),
                                      0.8f, 0.0f, 0.0f, qf.data(), out, false) == Status::ok,
            "unbraked pedaling ok");
    require(out.crank_torque_nm > 1.0f, "crank pedaling torque is nonzero when brakes are released");

    // 2. Standstill engage -> latches at current angle and applies spring bite on displacement
    qp[to_index(idx.front_wheel_spin_qpos)] = 1.25f;
    qv[to_index(idx.front_wheel_spin_dof)] = 0.2f;
    std::ranges::fill(qf, 0.0f);
    require(gpu_write_drivetrain_qfrc(dp, idx, st, qp.data(), qv.data(),
                                      0.0f, 0.8f, 0.0f, qf.data(), out, false) == Status::ok,
            "latch capture ok");
    require(out.front_brake_latched == 1 && out.rear_brake_latched == 0,
            "front brake latches below engage_radps when demand > 0");

    // Creep slightly past latched angle while stationary
    qp[to_index(idx.front_wheel_spin_qpos)] = 1.253f;
    qv[to_index(idx.front_wheel_spin_dof)] = 0.05f;
    std::ranges::fill(qf, 0.0f);
    std::array<float, kMaxNv> frictionloss{};
    require(gpu_write_drivetrain_qfrc(dp, idx, st, qp.data(), qv.data(),
                                      0.0f, 0.8f, 0.5f, qf.data(), out, false,
                                      frictionloss.data()) == Status::ok,
            "latched hold + static frictionloss ok");
    require_near(frictionloss[to_index(idx.front_wheel_spin_dof)], 160.0f, 1e-4f,
                 "apply_static_brake writes front frictionloss");
    require_near(frictionloss[to_index(idx.rear_wheel_spin_dof)], 100.0f, 1e-4f,
                 "apply_static_brake writes rear frictionloss");
    require(out.front_brake_torque_nm < -50.0f,
            "latched bite produces strong restoring hold torque against positive creep");
    // In static frictionloss mode, unlatched rear wheel omits dynamic torque
    require_near(out.rear_brake_torque_nm, 0.0f, 1e-6f,
                 "static frictionloss mode omits unlatched dynamic torque");

    // 3. Cadence taper above 105 rpm -> 0 at 120 rpm
    std::ranges::fill(qf, 0.0f);
    qv[to_index(idx.crank_spin_dof)] = dp.max_rider_cadence_rad_s;
    require(gpu_write_drivetrain_qfrc(dp, idx, st, qp.data(), qv.data(),
                                      1.0f, 0.0f, 0.0f, qf.data(), out, false) == Status::ok,
            "max cadence taper ok");
    require_near(out.crank_torque_nm, 0.0f, 1e-6f, "crank torque drops to zero at max cadence");
}

constexpr std::array<TestCase, 3> cases{{
    {.name = "resistance_parity_with_writer", .run = test_resistance_parity_with_writer},
    {.name = "ratchet_dual_and_freehub_parity", .run = test_ratchet_dual_and_freehub_parity},
    {.name = "brake_hold_and_dynamic_braking", .run = test_brake_hold_and_dynamic_braking},
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
            std::cerr << "usage: bike_test_gpu_drivetrain_resistance [case-name|--list]\n";
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
            std::cerr << "unknown gpu drivetrain_resistance case: " << requested << '\n';
            return 2;
        }
        return status;
    } catch (const std::exception &error) {
        std::cerr << "FAIL argument handling: " << error.what() << '\n';
        return 1;
    }
}
