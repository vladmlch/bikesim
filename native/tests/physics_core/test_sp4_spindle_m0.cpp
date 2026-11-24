#include "physics_core/common.hpp"
#include "physics_core/cpu_exact_toolkit.hpp"
#include "physics_core/gpu_math.hpp"
#include "physics_core/gpu_toolkit.hpp"
#include "physics_core/joint_envelope.hpp"
#include "physics_core/spindle_m0.hpp"
#include "physics_core/spindle_m2k2.hpp"
#include "physics_core/spindle_m2k2_rider_bridge.hpp"
#include "physics_core/spindle_m2k2_upload.hpp"

#include "configuration/load.hpp"
#include "configuration/model_config.hpp"
#include "configuration/rider_spec.hpp"
#include "engine_call.hpp"
#include "model_access.hpp"
#include "research/environment.hpp"
#include "rider/spindle_controller.hpp"
#include "rider/spindle_math.hpp"
#include "rider/state_codec.hpp"
#include "stepper.hpp"
#include "support/fixture_test.hpp"
#include "terrain/trackfile.hpp"
#include "validation/runtime_access.hpp"
#include "writers/drivetrain.hpp"
#include "writers/resistance.hpp"
#include "writers/rider_contacts.hpp"
#include "writers/suspension.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <numbers>
#include <optional>
#include <ranges>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

namespace fs = std::filesystem;
namespace pc = bike::physics_core;

static_assert(sizeof(pc::SpindleM0Scratch<float>) <= 4096,
              "SpindleM0Scratch<float> must stay compact (< 4 KB)");
static_assert(sizeof(pc::SpindleM0Scratch<double>) <= 4096,
              "SpindleM0Scratch<double> must stay compact (< 4 KB)");

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

std::size_t to_idx(int v) {
    if (v < 0) {
        throw std::out_of_range("negative index");
    }
    return static_cast<std::size_t>(v);
}

void require(bool cond, std::string_view msg) {
    bt::require(cond, msg);
}

void require_bits_f64(double actual, double expected, std::string_view what) {
    if (std::isnan(actual) && std::isnan(expected)) {
        return;
    }
    const auto ua = std::bit_cast<std::uint64_t>(actual);
    const auto ue = std::bit_cast<std::uint64_t>(expected);
    if (ua != ue) {
        std::ostringstream oss;
        oss << what << ": 64-bit mismatch (actual=" << actual << " [0x" << std::hex << ua
            << "], expected=" << expected << " [0x" << ue << "])";
        throw std::runtime_error(oss.str());
    }
}

void require_bits_f32(float actual, float expected, std::string_view what) {
    if (std::isnan(actual) && std::isnan(expected)) {
        return;
    }
    const auto ua = std::bit_cast<std::uint32_t>(actual);
    const auto ue = std::bit_cast<std::uint32_t>(expected);
    if (ua != ue) {
        std::ostringstream oss;
        oss << what << ": 32-bit mismatch (actual=0x" << std::hex << ua
            << ", expected=0x" << ue << ")";
        throw std::runtime_error(oss.str());
    }
}

fs::path repo_file(std::string_view rel) {
    return fs::path(BIKE_PHYSICS_CORE_SRC_DIR).parent_path().parent_path().parent_path() /
           fs::path(rel);
}

struct CpuExactTrig {
    static double flush_subnorm(double x) noexcept {
        return x;
    }
    template <class Real, class TK>
    static Real sin_val(Real x) noexcept {
        return static_cast<Real>(std::sin(static_cast<double>(x)));
    }
    template <class Real, class TK>
    static Real cos_val(Real x) noexcept {
        return static_cast<Real>(std::cos(static_cast<double>(x)));
    }
    template <class Real, class TK>
    static Real atan_val(Real x) noexcept {
        return static_cast<Real>(std::atan(static_cast<double>(x)));
    }
    template <class Real, class TK>
    static Real atan2_val(Real y, Real x) noexcept {
        return static_cast<Real>(std::atan2(static_cast<double>(y), static_cast<double>(x)));
    }
    template <class Real, class TK>
    static Real acos_val(Real x) noexcept {
        return static_cast<Real>(std::acos(std::clamp(static_cast<double>(x), -1.0, 1.0)));
    }
};

template <class Real>
std::vector<Real> to_real_vec(std::span<const double> src) {
    std::vector<Real> dst(src.size());
    for (std::size_t i = 0; i < src.size(); ++i) {
        dst[i] = static_cast<Real>(src[i]);
    }
    return dst;
}

template <class Real>
std::vector<Real> to_real_vec(const double *ptr, mjtSize count) {
    return to_real_vec<Real>(std::views::counted(ptr, count));
}

std::vector<pc::i32> to_i32_vec(const int *ptr, mjtSize count) {
    const auto sp = std::views::counted(ptr, count);
    return {sp.begin(), sp.end()};
}

double percentile_sorted(const std::vector<double> &sorted, double q) {
    if (sorted.empty()) {
        return 0.0;
    }
    const double idx = q * static_cast<double>(sorted.size() - 1);
    const auto lo = static_cast<std::size_t>(std::floor(idx));
    const auto hi = static_cast<std::size_t>(std::ceil(idx));
    const double frac = idx - static_cast<double>(lo);
    return sorted[lo] + frac * (sorted[hi] - sorted[lo]);
}

struct CapturedTestState {
    rider::RiderCommand cpu_cmd;
    bool steady_state = false;
    bool advance = true;
    bool pedal_recovery = true;
    std::optional<double> weld_crank_torque_nm;
    double initial_coast_hold_nm = 0.0;
    int step_index = 0;
    std::vector<double> qpos;
    std::vector<double> qvel;
    std::vector<double> qacc_warmstart;
    std::vector<double> ctrl;
    std::vector<double> qfrc_applied;
    std::vector<double> dof_frictionloss;
};

struct MultiStateCaptureBundle {
    ModelPtr model{nullptr, &delete_model};
    rider::SpindlePose pose;
    rider::SpindleConfig spindle_cfg;
    double crank_length_m = 0.165;
    int control_period_steps = 4;
    std::vector<CapturedTestState> states;
};

struct RideBootstrapSpec {
    std::string_view track_rel;
    double duration_s = 4.0;
    double initial_speed_mps = 0.0;
};

std::unique_ptr<runtime::RuntimeBootstrap> make_welded_ride_bootstrap(
    const RideBootstrapSpec &spec
) {
    auto owner = std::make_unique<bike::research::ResearchRequest>();
    auto &request = *owner;
    request.setup.model.physics = bike::configuration::load_physics(
        repo_file("examples/research/viewer_physics_welded.toml"));
    request.setup.model.physics->initial_speed_mps = spec.initial_speed_mps;
    request.setup.model.mode = "ride";
    request.setup.model.crank_joint = true;
    request.setup.model.rider = bike::model::rider_from_args("articulated_planar");
    request.setup.track = bike::terrain::load_track(repo_file(spec.track_rel));
    request.experiment.duration_s = spec.duration_s;
    request.experiment.record_decimation = 1;
    request.experiment.stop_on_model_violation = false;
    request.sensors = bike::research::SensorConfig{};
    request.experiment.control_period_s =
        request.setup.model.physics->research.control_period_s;
    request.experiment.actuator_delay_s =
        request.setup.model.physics->research.actuator_delay_s;
    request.sensors.delivery.latency_s =
        request.setup.model.physics->research.sensor_latency_s;
    request.sensors.delivery.maximum_age_s =
        request.setup.model.physics->research.sensor_maximum_age_s;
    request.sensors.sample_period_s = 0.005;
    auto prepared = bike::research::prepare_environment(std::move(owner));
    return std::make_unique<runtime::RuntimeBootstrap>(std::move(prepared->bootstrap.ride));
}

struct CaptureOptions {
    double mean_tq_override = 0.0;
    bool use_override = false;
    bool crank_repos = false;
    double target_rate = 0.0;
    int step_idx = 0;
};

MultiStateCaptureBundle capture_diverse_states_from_runtime() {
    MultiStateCaptureBundle out;
    auto boot_flat = make_welded_ride_bootstrap({.track_rel = "tests/fixtures/tracks/flat.toml"});
    out.control_period_steps = boot_flat->config.control_period_steps;
    out.crank_length_m = boot_flat->config.geometry.crank_length_m;
    out.pose = rider::decode_spindle_pose(boot_flat->config.geometry.pose, "config.geometry.pose");
    auto rt_flat = std::make_unique<runtime::NativeRideRuntime>(std::move(*boot_flat));
    boot_flat.reset();

    bike::validation::RuntimeRigAccess::inspect(
        *rt_flat, [&](const Stepper &stepper, const runtime::PhysicalStep &pstep) {
            mjModel *raw_copy = mj_copyModel(nullptr, stepper.model());
            if (raw_copy == nullptr) {
                throw std::runtime_error("mj_copyModel failed");
            }
            out.model = ModelPtr(raw_copy, &delete_model);
            out.spindle_cfg = pstep.rider_control().config();
        });

    const auto capture_from = [&](runtime::NativeRideRuntime &rt, const CaptureOptions &opts) {
        bike::validation::RuntimeRigAccess::inspect(
            rt, [&](const Stepper &stepper, const runtime::PhysicalStep &pstep) {
                const mjModel *m = stepper.model();
                const mjData *d = stepper.data();
                CapturedTestState st;
                const auto qpos_sp = model_access::readonly_buffer(d->qpos, m->nq, "qpos");
                const auto qvel_sp = model_access::readonly_buffer(d->qvel, m->nv, "qvel");
                const auto ws_sp = model_access::readonly_buffer(d->qacc_warmstart, m->nv, "warmstart");
                const auto ctrl_sp = model_access::readonly_buffer(d->ctrl, m->nu, "ctrl");
                const auto qfrc_sp = model_access::readonly_buffer(d->qfrc_applied, m->nv, "qfrc_applied");
                const auto fl_sp = model_access::readonly_buffer(m->dof_frictionloss, m->nv, "dof_frictionloss");

                st.qpos.assign(qpos_sp.begin(), qpos_sp.end());
                st.qvel.assign(qvel_sp.begin(), qvel_sp.end());
                st.qacc_warmstart.assign(ws_sp.begin(), ws_sp.end());
                st.ctrl.assign(ctrl_sp.begin(), ctrl_sp.end());
                st.qfrc_applied.assign(qfrc_sp.begin(), qfrc_sp.end());
                st.dof_frictionloss.assign(fl_sp.begin(), fl_sp.end());

                const auto &rc_state = pstep.rider_control().state();
                st.initial_coast_hold_nm = rc_state.coast_hold_nm.value_or(0.0);
                st.weld_crank_torque_nm = stepper.rider_contacts().state().delivered_crank_torque_nm;
                st.step_index = opts.step_idx;

                const auto &app = pstep.applied_control();
                st.cpu_cmd.enabled = app.rider_enabled;
                st.cpu_cmd.posture = app.posture.value_or(rider::RiderPosture{});
                st.cpu_cmd.crank_target_rate_rad_s =
                    app.crank_target_rate_rad_s.value_or(0.0);
                st.cpu_cmd.crank_reposition = opts.crank_repos;
                if (opts.crank_repos) {
                    st.cpu_cmd.mean_crank_torque_nm = 0.0;
                    st.cpu_cmd.crank_target_rate_rad_s = opts.target_rate;
                } else if (opts.use_override) {
                    st.cpu_cmd.mean_crank_torque_nm = opts.mean_tq_override;
                } else {
                    st.cpu_cmd.mean_crank_torque_nm = app.human_torque_nm.value_or(30.0);
                }
                out.states.push_back(std::move(st));
            });
    };

    runtime::RideControl ctrl_auto{};
    ctrl_auto.rider_enabled = true;

    runtime::RideControl ctrl_coast{};
    ctrl_coast.rider_enabled = true;
    ctrl_coast.human_torque_nm = 0.0;

    (void)rt_flat->advance(200, ctrl_auto, 0.0, 0.0, std::nullopt);
    for (int step = 220; step <= 1200; step += 28) {
        (void)rt_flat->advance(step, ctrl_auto, 0.0, 0.0, std::nullopt);
        capture_from(*rt_flat, {});
    }
    for (int step = 1220; step <= 1340; step += 8) {
        const bool do_coast = (step >= 1296);
        (void)rt_flat->advance(step, do_coast ? ctrl_coast : ctrl_auto, 0.0, 0.0, std::nullopt);
        if (step >= 1316) {
            capture_from(*rt_flat, {.use_override = true,
                                    .crank_repos = (step == 1332),
                                    .target_rate = (step == 1332) ? -3.0 : 0.0});
        } else {
            capture_from(*rt_flat, {});
        }
    }

    auto boot_climb = make_welded_ride_bootstrap({.track_rel = "tests/fixtures/tracks/climb_15pct.toml"});
    auto rt_climb = std::make_unique<runtime::NativeRideRuntime>(std::move(*boot_climb));
    boot_climb.reset();
    runtime::RideControl ctrl_climb{};
    ctrl_climb.rider_enabled = true;
    ctrl_climb.human_torque_nm = 60.0;
    (void)rt_climb->advance(160, ctrl_climb, 0.0, 0.0, std::nullopt);
    for (int step = 180; step <= 800; step += 40) {
        (void)rt_climb->advance(step, ctrl_climb, 0.0, 0.0, std::nullopt);
        capture_from(*rt_climb, {.mean_tq_override = 60.0, .use_override = true});
    }

    if (!out.states.empty()) {
        const CapturedTestState base_edge = out.states[out.states.size() / 2u];
        {
            CapturedTestState dis = base_edge;
            dis.cpu_cmd.enabled = false;
            out.states.push_back(std::move(dis));
        }
        {
            CapturedTestState ss = base_edge;
            ss.steady_state = true;
            ss.initial_coast_hold_nm = 3.5;
            out.states.push_back(std::move(ss));
        }
        {
            CapturedTestState noadv = base_edge;
            noadv.advance = false;
            noadv.initial_coast_hold_nm = 4.25;
            out.states.push_back(std::move(noadv));
        }
    }

    return out;
}

template <class Real>
pc::SpindleCommand<Real> make_spindle_cmd(const CapturedTestState &st) {
    pc::SpindleCommand<Real> cmd{};
    cmd.mean_crank_torque_nm = static_cast<Real>(st.cpu_cmd.mean_crank_torque_nm);
    cmd.torso_lean_rad = static_cast<Real>(st.cpu_cmd.posture.torso_lean_rad);
    cmd.crank_target_rate_rad_s = static_cast<Real>(st.cpu_cmd.crank_target_rate_rad_s);
    cmd.weld_crank_torque_nm = static_cast<Real>(st.weld_crank_torque_nm.value_or(0.0));
    cmd.has_weld_crank_torque = st.weld_crank_torque_nm.has_value() ? 1u : 0u;
    cmd.enabled = st.cpu_cmd.enabled ? 1u : 0u;
    cmd.crank_reposition = st.cpu_cmd.crank_reposition ? 1u : 0u;
    cmd.steady_state = st.steady_state ? 1u : 0u;
    cmd.advance = st.advance ? 1u : 0u;
    return cmd;
}

void copy_state_to_mjdata(const mjModel *m, mjData *d, const CapturedTestState &st) {
    mj_resetData(m, d);
    std::ranges::copy(st.qpos, model_access::mutable_buffer(d->qpos, m->nq, "qpos").begin());
    std::ranges::copy(st.qvel, model_access::mutable_buffer(d->qvel, m->nv, "qvel").begin());
    std::ranges::copy(st.qacc_warmstart,
                      model_access::mutable_buffer(d->qacc_warmstart, m->nv, "ws").begin());
    std::ranges::copy(st.ctrl, model_access::mutable_buffer(d->ctrl, m->nu, "ctrl").begin());
    std::ranges::copy(st.qfrc_applied,
                      model_access::mutable_buffer(d->qfrc_applied, m->nv, "qfrc").begin());
    std::ranges::copy(st.dof_frictionloss,
                      model_access::mutable_buffer(m->dof_frictionloss, m->nv, "fl").begin());
    engine::forward(m, d);
}

// ---- 1. spindle_m0.hpp<double> vs CPU SpindleController(m0) ----------------

void test_spindle_m0_fp64_parity(const MultiStateCaptureBundle &bundle) {
    std::cout << "[SP4-M0] 1. spindle_m0.hpp<double> vs CPU SpindleController(m0) across "
              << bundle.states.size() << " states...\n";

    const ModelPtr m_mut(mj_copyModel(nullptr, bundle.model.get()), &delete_model);
    mjModel *m = m_mut.get();

    const pc::SpindleUploadTiming timing{
        .crank_length_m = bundle.crank_length_m,
        .control_period_steps = bundle.control_period_steps,
    };
    auto params64_ptr = std::make_unique<pc::SpindleM0Params<double>>(
        pc::build_spindle_m2k2_params_from_spindle_config<double>(
            m, bundle.pose, bundle.spindle_cfg, timing));
    params64_ptr->ik_reach_eps = 1e-10;
    params64_ptr->ik_dir_eps = 1e-15;
    params64_ptr->det_eps = 1e-12;
    const auto &params64 = *params64_ptr;

    auto ctrl_cpu = std::make_unique<rider::SpindleController>(
        m, bundle.pose, bundle.spindle_cfg, bundle.crank_length_m);
    require(ctrl_cpu->solver_mode() == rider::SpindleSolverMode::m0,
            "default solver_mode is m0");

    DataPtr d_cpu(engine::make_data(m), &delete_data);
    DataPtr d_m0(engine::make_data(m), &delete_data);

    const auto M_rowadr_i32 = to_i32_vec(m->M_rowadr, m->nv);
    const auto M_rownnz_i32 = to_i32_vec(m->M_rownnz, m->nv);
    const auto M_colind_i32 = to_i32_vec(m->M_colind, static_cast<int>(m->nC));

    std::vector<double> all_tau_errs;
    std::vector<double> leg_tau_errs;
    std::vector<double> upper_tau_errs;
    double max_hold_err = 0.0;
    int max_solves = 0;
    int handover_count = 0;

    auto m0_scratch = std::make_unique<pc::SpindleM0Scratch<double>>();

    for (const auto &st : bundle.states) {

        copy_state_to_mjdata(m, d_cpu.get(), st);
        {
            const auto xmat = model_access::readonly_buffer(d_cpu->xmat, m->nbody * 9, "xmat");
            const auto xpos = model_access::readonly_buffer(d_cpu->xpos, m->nbody * 3, "xpos");
            const auto xanchor = model_access::readonly_buffer(d_cpu->xanchor, m->njnt * 3, "xanchor");
            const std::size_t fb9 = to_idx(params64.frame_body) * 9;
            const std::size_t cj3 = to_idx(params64.crank_joint) * 3;
            const std::size_t pb3 = to_idx(params64.pedal_bodies[0]) * 3;
            const double dx = xpos[pb3 + 0] - xanchor[cj3 + 0];
            const double dy = xpos[pb3 + 1] - xanchor[cj3 + 1];
            const double dz = xpos[pb3 + 2] - xanchor[cj3 + 2];
            const double spx = xmat[fb9 + 0] * dx + xmat[fb9 + 3] * dy + xmat[fb9 + 6] * dz;
            const double spz = xmat[fb9 + 2] * dx + xmat[fb9 + 5] * dy + xmat[fb9 + 8] * dz;
            const double phase = std::atan2(-spz, spx);
            if (std::abs(std::cos(phase)) <= 0.25) {
                ++handover_count;
            }
        }

        auto cstate = ctrl_cpu->state();
        cstate.coast_hold_nm = st.initial_coast_hold_nm;
        ctrl_cpu->restore(cstate);

        const auto cpu_torques = ctrl_cpu->compute(
            d_cpu.get(), st.cpu_cmd, st.advance, std::nullopt,
            st.steady_state, std::nullopt, st.pedal_recovery, st.weld_crank_torque_nm);
        ctrl_cpu->write(d_cpu.get(), cpu_torques);
        const double cpu_hold_after = ctrl_cpu->state().coast_hold_nm.value_or(0.0);

        // Run spindle_m0.hpp<double> driven by mj_forwardSkip
        copy_state_to_mjdata(m, d_m0.get(), st);
        const auto cmd64 = make_spindle_cmd<double>(st);
        pc::SpindleM0State<double> m0_state{};
        m0_state.coast_hold_nm = st.initial_coast_hold_nm;
        *m0_scratch = pc::SpindleM0Scratch<double>{};

        std::vector<double> ctrl64(
            model_access::readonly_buffer(d_m0->ctrl, m->nu, "c").begin(),
            model_access::readonly_buffer(d_m0->ctrl, m->nu, "c").end());
        const auto write_d_m0_ctrl = [&]() {
            const auto dst = model_access::mutable_buffer(d_m0->ctrl, m->nu, "c");
            for (int a = 0; a < m->nu; ++a) {
                dst[to_idx(a)] = ctrl64[to_idx(a)];
            }
        };

        pc::u8 needs_solve = 0;
        pc::u8 done = 0;
        pc::spindle_m0_begin<double, pc::CpuExactToolkit, CpuExactTrig>(
            &params64, cmd64, st.step_index,
            d_m0->qpos, d_m0->qvel, d_m0->qfrc_bias,
            d_m0->xmat, d_m0->xpos, d_m0->xanchor, d_m0->xaxis,
            d_m0->site_xpos, d_m0->subtree_com, m->body_subtreemass,
            M_rowadr_i32.data(), M_rownnz_i32.data(), M_colind_i32.data(), d_m0->M,
            &m0_state, m0_scratch.get(), ctrl64.data(), needs_solve, done);

        int solves = 0;
        while (done == 0) {
            require(needs_solve != 0, "unfinished M0 state must request solve");
            require(solves < pc::kM0MaxPasses, "M0 solve count within kM0MaxPasses");
            write_d_m0_ctrl();
            mj_forwardSkip(m, d_m0.get(), mjSTAGE_VEL, 1);
            ++solves;
            pc::spindle_m0_step<double, pc::CpuExactToolkit, CpuExactTrig>(
                &params64, d_m0->qpos, d_m0->qvel, d_m0->qfrc_bias, d_m0->qacc,
                M_rowadr_i32.data(), M_rownnz_i32.data(), M_colind_i32.data(), d_m0->M,
                &m0_state, m0_scratch.get(), ctrl64.data(), needs_solve, done);
        }
        require(needs_solve == 0, "done M0 state must clear needs_solve");
        max_solves = std::max(max_solves, solves);

        // Verify calling spindle_m0_step after done==1 is a clean no-op
        const auto ctrl64_snap = ctrl64;
        const auto hold_snap = m0_state.coast_hold_nm;
        pc::spindle_m0_step<double, pc::CpuExactToolkit, CpuExactTrig>(
            &params64, d_m0->qpos, d_m0->qvel, d_m0->qfrc_bias, d_m0->qacc,
            M_rowadr_i32.data(), M_rownnz_i32.data(), M_colind_i32.data(), d_m0->M,
            &m0_state, m0_scratch.get(), ctrl64.data(), needs_solve, done);
        require(done == 1 && needs_solve == 0, "post-done step stays done");
        for (int a = 0; a < m->nu; ++a) {
            require_bits_f64(ctrl64[to_idx(a)], ctrl64_snap[to_idx(a)], "post-done ctrl unchanged");
        }
        require_bits_f64(m0_state.coast_hold_nm, hold_snap, "post-done hold unchanged");

        if (st.advance) {
            max_hold_err = std::max(max_hold_err, std::abs(m0_state.coast_hold_nm - cpu_hold_after));
        } else {
            require_bits_f64(m0_state.coast_hold_nm, st.initial_coast_hold_nm,
                             "M0 advance=0 must not mutate coast_hold_nm");
        }

        const auto d_cpu_ctrl = model_access::readonly_buffer(d_cpu->ctrl, m->nu, "ctrl");
        for (int i = 0; i < pc::kNumSpindleJoints; ++i) {
            const int aid = std::span{params64.joint_actuator_id}[to_idx(i)];
            const double err = std::abs(ctrl64[to_idx(aid)] - d_cpu_ctrl[to_idx(aid)]);
            all_tau_errs.push_back(err);
            if (i < pc::kNumUpperSpindleJoints) {
                upper_tau_errs.push_back(err);
            } else {
                leg_tau_errs.push_back(err);
            }
        }
    }

    std::ranges::sort(all_tau_errs);
    std::ranges::sort(leg_tau_errs);
    std::ranges::sort(upper_tau_errs);
    std::cout << "  Handover states=" << handover_count
              << " | max_solves=" << max_solves
              << " | M0_F64 vs CPU(M0) |dTau_all| p50=" << percentile_sorted(all_tau_errs, 0.50)
              << " p95=" << percentile_sorted(all_tau_errs, 0.95)
              << " max=" << all_tau_errs.back()
              << " (leg max=" << leg_tau_errs.back()
              << ", upper max=" << upper_tau_errs.back()
              << ", hold max=" << max_hold_err << ")\n";

    // Note: CPU SpindleController(m0) computes djacs via kJacobianDelta=1e-3 FD
    // whereas physics_core uses closed-form analytic_leg_kinematics (~1e-6..1e-5 djac difference),
    // eliminating the 17.71-19.42 Nm M2_K2 handover gap down to ~1e-5 Nm!
    require(handover_count >= 2, "captured handover states");
    require(all_tau_errs.back() < 1e-4, "M0_F64 vs CPU(M0) max |dTau| < 1e-4 Nm (zero 17.71 Nm gap)");
    require(upper_tau_errs.back() < 1e-7, "M0_F64 upper |dTau| < 1e-7 Nm");
    require(max_hold_err < 1e-5, "M0_F64 coast_hold_nm parity < 1e-5 Nm");
}

// ---- 2. SpindleController(m2k2) vs spindle_m2k2.hpp<double> 0-bit parity ----

void test_spindle_controller_m2k2_zero_bit_parity(const MultiStateCaptureBundle &bundle) {
    std::cout << "[SP4-M0] 2. SpindleController(m2k2) vs spindle_m2k2.hpp<double> 0-bit parity...\n";

    const ModelPtr m_mut(mj_copyModel(nullptr, bundle.model.get()), &delete_model);
    mjModel *m = m_mut.get();

    const pc::SpindleUploadTiming timing{
        .crank_length_m = bundle.crank_length_m,
        .control_period_steps = 1,
    };
    const auto params64_ptr = std::make_unique<pc::SpindleM2K2Params<double>>(
        pc::build_spindle_m2k2_params_from_spindle_config<double>(
            m, bundle.pose, bundle.spindle_cfg, timing));
    const auto &params64 = *params64_ptr;

    auto ctrl_m2k2 = std::make_unique<rider::SpindleController>(
        m, bundle.pose, bundle.spindle_cfg, bundle.crank_length_m);
    ctrl_m2k2->set_solver_mode(rider::SpindleSolverMode::m2k2);
    require(ctrl_m2k2->solver_mode() == rider::SpindleSolverMode::m2k2,
            "solver_mode switched to m2k2");

    DataPtr d_cpu(engine::make_data(m), &delete_data);
    DataPtr d_ref(engine::make_data(m), &delete_data);

    const auto M_rowadr_i32 = to_i32_vec(m->M_rowadr, m->nv);
    const auto M_rownnz_i32 = to_i32_vec(m->M_rownnz, m->nv);
    const auto M_colind_i32 = to_i32_vec(m->M_colind, static_cast<int>(m->nC));

    auto ref_state = std::make_unique<pc::SpindleM2K2State<double>>();
    auto ref_scratch = std::make_unique<pc::SpindleM2K2StepScratch<double>>();

    // Run states sequentially (carrying prev_ps across ticks!) AND test restore/reset
    for (std::size_t si = 0; si < bundle.states.size(); ++si) {
        const auto &st = bundle.states[si];

        copy_state_to_mjdata(m, d_cpu.get(), st);
        copy_state_to_mjdata(m, d_ref.get(), st);

        if (si == 0 || !st.advance) {
            auto cs = ctrl_m2k2->state();
            cs.coast_hold_nm = st.initial_coast_hold_nm;
            ctrl_m2k2->restore(cs);

            ref_state->coast_hold_nm = st.initial_coast_hold_nm;
            ref_state->has_held_torques = 0;
            ref_state->has_prev_ps = 0;
            ref_state->prev_ps = pc::PressStateCore<double>{};
        }

        const auto cpu_torques = ctrl_m2k2->compute(
            d_cpu.get(), st.cpu_cmd, st.advance, std::nullopt,
            st.steady_state, std::nullopt, st.pedal_recovery, st.weld_crank_torque_nm);
        ctrl_m2k2->write(d_cpu.get(), cpu_torques);

        const auto cmd64 = make_spindle_cmd<double>(st);
        std::vector<double> ctrl_ref(
            model_access::readonly_buffer(d_ref->ctrl, m->nu, "c").begin(),
            model_access::readonly_buffer(d_ref->ctrl, m->nu, "c").end());
        const auto write_ref_ctrl = [&]() {
            const auto dst = model_access::mutable_buffer(d_ref->ctrl, m->nu, "c");
            for (int a = 0; a < m->nu; ++a) {
                dst[to_idx(a)] = ctrl_ref[to_idx(a)];
            }
        };

        *ref_scratch = pc::SpindleM2K2StepScratch<double>{};
        pc::spindle_m2k2_pass0_init_and_base<double, pc::CpuExactToolkit, CpuExactTrig>(
            &params64, cmd64, /*step_index=*/0,
            d_ref->qpos, d_ref->qvel, d_ref->qfrc_bias,
            d_ref->xmat, d_ref->xpos, d_ref->xanchor, d_ref->xaxis,
            d_ref->site_xpos, d_ref->subtree_com, m->body_subtreemass,
            M_rowadr_i32.data(), M_rownnz_i32.data(), M_colind_i32.data(), d_ref->M,
            ref_state.get(), ref_scratch.get(), ctrl_ref.data());

        if (ref_scratch->active_newton != 0) {
            write_ref_ctrl();
            mj_forwardSkip(m, d_ref.get(), mjSTAGE_VEL, 1);
            pc::spindle_m2k2_pass1_perturb0<double, pc::CpuExactToolkit, CpuExactTrig>(
                &params64, d_ref->qpos, d_ref->qvel, d_ref->qfrc_bias, d_ref->qacc,
                M_rowadr_i32.data(), M_rownnz_i32.data(), M_colind_i32.data(), d_ref->M,
                ref_state.get(), ref_scratch.get(), ctrl_ref.data());
            write_ref_ctrl();
            mj_forwardSkip(m, d_ref.get(), mjSTAGE_VEL, 1);
            pc::spindle_m2k2_pass2_perturb1<double, pc::CpuExactToolkit, CpuExactTrig>(
                &params64, d_ref->qpos, d_ref->qvel, d_ref->qfrc_bias, d_ref->qacc,
                M_rowadr_i32.data(), M_rownnz_i32.data(), M_colind_i32.data(), d_ref->M,
                ref_state.get(), ref_scratch.get(), ctrl_ref.data());
            write_ref_ctrl();
            mj_forwardSkip(m, d_ref.get(), mjSTAGE_VEL, 1);
            pc::spindle_m2k2_pass3_commit<double, pc::CpuExactToolkit, CpuExactTrig>(
                &params64, d_ref->qpos, d_ref->qvel, d_ref->qfrc_bias, d_ref->qacc,
                M_rowadr_i32.data(), M_rownnz_i32.data(), M_colind_i32.data(), d_ref->M,
                ref_state.get(), ref_scratch.get(), ctrl_ref.data());
        }

        const auto d_cpu_ctrl = model_access::readonly_buffer(d_cpu->ctrl, m->nu, "ctrl");
        for (int i = 0; i < pc::kNumSpindleJoints; ++i) {
            const int aid = std::span{params64.joint_actuator_id}[to_idx(i)];
            require_bits_f64(cpu_torques[to_idx(i)].second, ctrl_ref[to_idx(aid)],
                             "SpindleController(m2k2) return 0-bit vs spindle_m2k2.hpp");
            require_bits_f64(d_cpu_ctrl[to_idx(aid)], ctrl_ref[to_idx(aid)],
                             "SpindleController(m2k2) write() 0-bit vs spindle_m2k2.hpp");
        }
        require_bits_f64(ctrl_m2k2->state().coast_hold_nm.value_or(0.0),
                         ref_state->coast_hold_nm,
                         "SpindleController(m2k2) coast_hold_nm 0-bit parity");

        const auto &ik = ctrl_m2k2->presentation_ik();
        const auto m2k2_diag_state = ctrl_m2k2->state();
        const auto &alloc = m2k2_diag_state.allocation_diagnostics;
        const auto &eff = m2k2_diag_state.effort_diagnostics;
        const auto &terms = ctrl_m2k2->presentation_terms();
        if (!st.cpu_cmd.enabled) {
            require(ik.empty(), "SpindleController(m2k2) disabled command clears saturated_ik");
            require_bits_f64(alloc.crank_task_nm, 0.0,
                             "SpindleController(m2k2) disabled crank_task_nm");
            require_bits_f64(alloc.crank_task_shortfall_nm, 0.0,
                             "SpindleController(m2k2) disabled crank_task_shortfall_nm");
        } else {
            require(ik.size() == 2 && ik[0].first == "torso" && ik[1].first == "arms",
                    "SpindleController(m2k2) saturated_ik keys");
            require(ik[0].second == (ref_scratch->torso_saturated != 0),
                    "SpindleController(m2k2) torso_saturated parity");
            require(ik[1].second == (ref_scratch->arms_saturated != 0),
                    "SpindleController(m2k2) arms_saturated parity");
            require_bits_f64(alloc.crank_task_nm, ref_scratch->crank_task_nm,
                             "SpindleController(m2k2) crank_task_nm 0-bit parity");
        }
        require(alloc.present && !alloc.invalid_controller,
                "SpindleController(m2k2) allocation_diagnostics present");
        require(std::isfinite(alloc.crank_task_shortfall_nm),
                "SpindleController(m2k2) crank_task_shortfall_nm finite");
        require(eff.present, "SpindleController(m2k2) effort_diagnostics present");
        require(terms.size() == static_cast<std::size_t>(pc::kNumSpindleJoints),
                "SpindleController(m2k2) last_terms width");
        const std::span ref_req{ref_scratch->last_req};
        const std::span ref_exc{ref_scratch->last_exc};
        for (int i = 0; i < pc::kNumSpindleJoints; ++i) {
            const int aid = std::span{params64.joint_actuator_id}[to_idx(i)];
            const std::size_t slot = to_idx(i);
            require_bits_f64(terms[slot].second.requested_nm,
                             ref_req[slot],
                             "SpindleController(m2k2) terms.requested_nm parity");
            if (!terms[slot].second.active_request_nm.has_value() ||
                !terms[slot].second.active_delivered_nm.has_value()) {
                throw std::runtime_error("SpindleController(m2k2) terms active fields set");
            }
            require_bits_f64(*terms[slot].second.active_request_nm,
                             ref_exc[slot],
                             "SpindleController(m2k2) terms.active_request_nm parity");
            require_bits_f64(*terms[slot].second.active_delivered_nm,
                             ctrl_ref[to_idx(aid)],
                             "SpindleController(m2k2) terms.active_delivered_nm parity");
        }
    }

    // Verify switching back to m0 preserves default CPU(m0) 0-bit parity
    // and compare saturated_ik / diagnostics under extreme lean + high torque.
    auto ctrl_default = std::make_unique<rider::SpindleController>(
        m, bundle.pose, bundle.spindle_cfg, bundle.crank_length_m);
    const auto &probe = bundle.states[bundle.states.size() / 2u];
    {
        auto ext_cmd = probe.cpu_cmd;
        ext_cmd.enabled = true;
        ext_cmd.posture.torso_lean_rad = 1.5;
        copy_state_to_mjdata(m, d_cpu.get(), probe);
        copy_state_to_mjdata(m, d_ref.get(), probe);
        (void)ctrl_m2k2->compute(
            d_cpu.get(), ext_cmd, /*advance=*/false, std::nullopt,
            /*steady_state=*/false, std::nullopt, false, probe.weld_crank_torque_nm);
        (void)ctrl_default->compute(
            d_ref.get(), ext_cmd, /*advance=*/false, std::nullopt,
            /*steady_state=*/false, std::nullopt, false, probe.weld_crank_torque_nm);
        const auto &ik_m2 = ctrl_m2k2->presentation_ik();
        const auto &ik_m0 = ctrl_default->presentation_ik();
        require(ik_m2 == ik_m0,
                "SpindleController(m2k2) extreme-lean saturated_ik matches M0");
        require(ik_m2.size() == 2 && (ik_m2[0].second || ik_m2[1].second),
                "extreme lean triggers non-trivial IK saturation");
    }
    {
        rider::SpindleConfig tight_cfg = bundle.spindle_cfg;
        tight_cfg.has_strength = false;
        tight_cfg.joint_limit_nm = 5.0;
        tight_cfg.joint_power_limit_w = 10.0;
        auto tight_m2 = std::make_unique<rider::SpindleController>(
            m, bundle.pose, tight_cfg, bundle.crank_length_m);
        tight_m2->set_solver_mode(rider::SpindleSolverMode::m2k2);
        auto tight_m0 = std::make_unique<rider::SpindleController>(
            m, bundle.pose, tight_cfg, bundle.crank_length_m);
        auto tight_cmd = probe.cpu_cmd;
        tight_cmd.enabled = true;
        tight_cmd.mean_crank_torque_nm = 100.0;
        copy_state_to_mjdata(m, d_cpu.get(), probe);
        copy_state_to_mjdata(m, d_ref.get(), probe);
        (void)tight_m2->compute(
            d_cpu.get(), tight_cmd, /*advance=*/false, std::nullopt,
            /*steady_state=*/false, std::nullopt, false, probe.weld_crank_torque_nm);
        (void)tight_m0->compute(
            d_ref.get(), tight_cmd, /*advance=*/false, std::nullopt,
            /*steady_state=*/false, std::nullopt, false, probe.weld_crank_torque_nm);
        const auto st_m2 = tight_m2->state();
        const auto st_m0 = tight_m0->state();
        require(st_m2.allocation_diagnostics.crank_task_nm > 1.0 &&
                    st_m2.allocation_diagnostics.crank_task_shortfall_nm > 1.0,
                "tight torque/power ceiling produces positive crank task & shortfall");
        require(std::abs(st_m2.allocation_diagnostics.crank_task_shortfall_nm -
                         st_m0.allocation_diagnostics.crank_task_shortfall_nm) < 0.5,
                "tight ceiling shortfall matches M0 within 0.5 Nm");
        const bool any_sat = std::ranges::any_of(
            tight_m2->presentation_terms(),
            [](const auto &kv) { return kv.second.saturated; });
        require(any_sat, "tight ceiling marks saturated joint term(s)");
    }

    ctrl_m2k2->set_solver_mode(rider::SpindleSolverMode::m0);
    copy_state_to_mjdata(m, d_cpu.get(), probe);
    copy_state_to_mjdata(m, d_ref.get(), probe);
    const auto t_switched = ctrl_m2k2->compute(
        d_cpu.get(), probe.cpu_cmd, probe.advance, std::nullopt,
        probe.steady_state, std::nullopt, probe.pedal_recovery, probe.weld_crank_torque_nm);
    const auto t_def = ctrl_default->compute(
        d_ref.get(), probe.cpu_cmd, probe.advance, std::nullopt,
        probe.steady_state, std::nullopt, probe.pedal_recovery, probe.weld_crank_torque_nm);
    for (std::size_t i = 0; i < t_def.size(); ++i) {
        require_bits_f64(t_switched[i].second, t_def[i].second,
                         "SpindleController switched back to m0 matches default 0-bit");
    }

    std::cout << "  0-bit parity verified across all " << bundle.states.size()
              << " sequential states (including prev_ps carry, steady_state, disabled, advance=false).\n";
}

// ---- 3. spindle_m0.hpp<float> multi-world batch isolation & kEps study ----

void test_spindle_m0_fp32_multiworld_and_eps(const MultiStateCaptureBundle &bundle) {
    std::cout << "[SP4-M0] 3. spindle_m0.hpp<float> multi-world batch isolation & kEps=1e-3f vs 1e-2f...\n";

    const ModelPtr m_mut(mj_copyModel(nullptr, bundle.model.get()), &delete_model);
    mjModel *m = m_mut.get();

    const pc::SpindleUploadTiming timing{
        .crank_length_m = bundle.crank_length_m,
        .control_period_steps = bundle.control_period_steps,
    };
    const auto params32_ptr = std::make_unique<pc::SpindleM0Params<float>>(
        pc::build_spindle_m2k2_params_from_spindle_config<float>(
            m, bundle.pose, bundle.spindle_cfg, timing));
    const auto &params32 = *params32_ptr;

    const auto M_rowadr_i32 = to_i32_vec(m->M_rowadr, m->nv);
    const auto M_rownnz_i32 = to_i32_vec(m->M_rownnz, m->nv);
    const auto M_colind_i32 = to_i32_vec(m->M_colind, static_cast<int>(m->nC));
    const auto submass_f32 = to_real_vec<float>(m->body_subtreemass, m->nbody);

    auto ctrl_cpu = std::make_unique<rider::SpindleController>(
        m, bundle.pose, bundle.spindle_cfg, bundle.crank_length_m);
    DataPtr d_cpu(engine::make_data(m), &delete_data);

    // Reference CPU(m0) torques for all states
    std::vector<std::array<double, pc::kNumSpindleJoints>> cpu_ctrl_all(bundle.states.size());
    for (std::size_t w = 0; w < bundle.states.size(); ++w) {
        const auto &st = bundle.states[w];
        copy_state_to_mjdata(m, d_cpu.get(), st);
        auto cs = ctrl_cpu->state();
        cs.coast_hold_nm = st.initial_coast_hold_nm;
        ctrl_cpu->restore(cs);
        const auto tq = ctrl_cpu->compute(
            d_cpu.get(), st.cpu_cmd, st.advance, std::nullopt,
            st.steady_state, std::nullopt, st.pedal_recovery, st.weld_crank_torque_nm);
        ctrl_cpu->write(d_cpu.get(), tq);
        const auto dc = model_access::readonly_buffer(d_cpu->ctrl, m->nu, "c");
        for (int i = 0; i < pc::kNumSpindleJoints; ++i) {
            const int aid = std::span{params32.joint_actuator_id}[to_idx(i)];
            cpu_ctrl_all[w][to_idx(i)] = dc[to_idx(aid)];
        }
    }

    static_assert(pc::SpindleM0Options<float>{}.newton_eps == 1e-2f);
    static_assert(pc::SpindleM0Options<float>{}.conv_tol == 1e-4f);
    static_assert(pc::SpindleM0Options<float>{}.det_eps == 1e-12f);
    static_assert(pc::SpindleM0Options<double>{}.newton_eps == 1e-3);
    static_assert(pc::SpindleM0Options<double>{}.conv_tol == 1e-6);
    static_assert(pc::SpindleM0Options<double>{}.det_eps == 1e-12);

    const auto run_batched_fp32 = [&](std::optional<pc::SpindleM0Options<float>> opts_opt,
                                      std::string_view label) {
        const std::size_t nworld = bundle.states.size();
        std::vector<DataPtr> d_worlds;
        d_worlds.reserve(nworld);
        std::vector<pc::SpindleM0State<float>> st_worlds(nworld);
        std::vector<pc::SpindleM0Scratch<float>> scr_worlds(nworld);
        std::vector<std::vector<float>> ctrl_worlds(nworld);
        std::vector<pc::u8> needs_solve(nworld, 0u);
        std::vector<pc::u8> done(nworld, 0u);
        std::vector<int> world_solves(nworld, 0);
        std::vector<std::vector<float>> done_ctrl_snapshot(nworld);
        std::vector<pc::u8> has_done_snapshot(nworld, 0u);

        for (std::size_t w = 0; w < nworld; ++w) {
            const auto &st = bundle.states[w];
            DataPtr dw(engine::make_data(m), &delete_data);
            copy_state_to_mjdata(m, dw.get(), st);
            st_worlds[w].coast_hold_nm = static_cast<float>(st.initial_coast_hold_nm);

            const auto qpos_f32 = to_real_vec<float>(dw->qpos, m->nq);
            const auto qvel_f32 = to_real_vec<float>(dw->qvel, m->nv);
            const auto bias_f32 = to_real_vec<float>(dw->qfrc_bias, m->nv);
            const auto xmat_f32 = to_real_vec<float>(dw->xmat, m->nbody * 9);
            const auto xpos_f32 = to_real_vec<float>(dw->xpos, m->nbody * 3);
            const auto xanc_f32 = to_real_vec<float>(dw->xanchor, m->njnt * 3);
            const auto xax_f32 = to_real_vec<float>(dw->xaxis, m->njnt * 3);
            const auto spos_f32 = to_real_vec<float>(dw->site_xpos, m->nsite * 3);
            const auto scom_f32 = to_real_vec<float>(dw->subtree_com, m->nbody * 3);
            const auto dM_f32 = to_real_vec<float>(dw->M, static_cast<int>(m->nC));
            ctrl_worlds[w] = to_real_vec<float>(dw->ctrl, m->nu);

            pc::spindle_m0_begin<float, pc::GpuToolkit>(
                &params32, make_spindle_cmd<float>(st), st.step_index,
                qpos_f32.data(), qvel_f32.data(), bias_f32.data(),
                xmat_f32.data(), xpos_f32.data(), xanc_f32.data(), xax_f32.data(),
                spos_f32.data(), scom_f32.data(), submass_f32.data(),
                M_rowadr_i32.data(), M_rownnz_i32.data(), M_colind_i32.data(), dM_f32.data(),
                &st_worlds[w], &scr_worlds[w], ctrl_worlds[w].data(),
                needs_solve[w], done[w]);
            if (done[w] != 0) {
                done_ctrl_snapshot[w] = ctrl_worlds[w];
                has_done_snapshot[w] = 1u;
            }
            d_worlds.push_back(std::move(dw));
        }

        // Batched synchronous passes across all worlds simultaneously:
        // Fast-converging / steady / disabled worlds already have done[w]==1,
        // while slow handover worlds iterate; verify done worlds remain bit-frozen!
        int pass = 0;
        for (; pass < pc::kM0MaxPasses; ++pass) {
            bool any_needs = false;
            for (std::size_t w = 0; w < nworld; ++w) {
                if (needs_solve[w] != 0) {
                    any_needs = true;
                    const auto dst = model_access::mutable_buffer(
                        d_worlds[w]->ctrl, m->nu, "c");
                    for (int a = 0; a < m->nu; ++a) {
                        dst[to_idx(a)] = static_cast<double>(ctrl_worlds[w][to_idx(a)]);
                    }
                    mj_forwardSkip(m, d_worlds[w].get(), mjSTAGE_VEL, 1);
                    ++world_solves[w];
                }
            }
            if (!any_needs) {
                break;
            }
            // Deliberately invoke spindle_m0_step on ALL worlds (including done ones)
            // to prove multi-world batch isolation.
            for (std::size_t w = 0; w < nworld; ++w) {
                const mjData *dw = d_worlds[w].get();
                const auto qpos_f32 = to_real_vec<float>(dw->qpos, m->nq);
                const auto qvel_f32 = to_real_vec<float>(dw->qvel, m->nv);
                const auto bias_f32 = to_real_vec<float>(dw->qfrc_bias, m->nv);
                const auto qacc_f32 = to_real_vec<float>(dw->qacc, m->nv);
                const auto dM_f32 = to_real_vec<float>(dw->M, static_cast<int>(m->nC));

                if (opts_opt.has_value()) {
                    pc::spindle_m0_step<float, pc::GpuToolkit>(
                        &params32, qpos_f32.data(), qvel_f32.data(), bias_f32.data(), qacc_f32.data(),
                        M_rowadr_i32.data(), M_rownnz_i32.data(), M_colind_i32.data(), dM_f32.data(),
                        &st_worlds[w], &scr_worlds[w], ctrl_worlds[w].data(),
                        needs_solve[w], done[w], *opts_opt);
                } else {
                    pc::spindle_m0_step<float, pc::GpuToolkit>(
                        &params32, qpos_f32.data(), qvel_f32.data(), bias_f32.data(), qacc_f32.data(),
                        M_rowadr_i32.data(), M_rownnz_i32.data(), M_colind_i32.data(), dM_f32.data(),
                        &st_worlds[w], &scr_worlds[w], ctrl_worlds[w].data(),
                        needs_solve[w], done[w]);
                }

                if (has_done_snapshot[w] != 0) {
                    require(done[w] == 1 && needs_solve[w] == 0,
                            "already-done world must remain done in batch");
                    for (int a = 0; a < m->nu; ++a) {
                        require_bits_f32(ctrl_worlds[w][to_idx(a)],
                                         done_ctrl_snapshot[w][to_idx(a)],
                                         "already-done world ctrl preserved bit-for-bit");
                    }
                } else if (done[w] != 0) {
                    done_ctrl_snapshot[w] = ctrl_worlds[w];
                    has_done_snapshot[w] = 1u;
                }
            }
        }

        for (std::size_t w = 0; w < nworld; ++w) {
            require(done[w] == 1 && needs_solve[w] == 0, "all worlds finished within kM0MaxPasses");
        }

        std::vector<double> errs;
        int min_s = 1000;
        int max_s = 0;
        for (std::size_t w = 0; w < nworld; ++w) {
            min_s = std::min(min_s, world_solves[w]);
            max_s = std::max(max_s, world_solves[w]);
            for (int i = 0; i < pc::kNumSpindleJoints; ++i) {
                const int aid = std::span{params32.joint_actuator_id}[to_idx(i)];
                errs.push_back(std::abs(
                    static_cast<double>(ctrl_worlds[w][to_idx(aid)]) - cpu_ctrl_all[w][to_idx(i)]));
            }
        }
        std::ranges::sort(errs);
        const double p50 = percentile_sorted(errs, 0.50);
        const double p95 = percentile_sorted(errs, 0.95);
        std::cout << "  " << label
                  << ": passes=" << pass << " solves=[" << min_s << ".." << max_s << "]"
                  << " |dTau| p50=" << p50
                  << " p95=" << p95
                  << " max=" << errs.back() << " Nm\n";
        return std::array<double, 3>{p50, p95, errs.back()};
    };

    const auto err_1e3_tol1e4 = run_batched_fp32(
        pc::SpindleM0Options<float>{.newton_eps = 1e-3f, .conv_tol = 1e-4f, .det_eps = 1e-12f},
        "kEps=1e-3f, tol=1e-4f");
    const auto err_1e3_tol1e5 = run_batched_fp32(
        pc::SpindleM0Options<float>{.newton_eps = 1e-3f, .conv_tol = 1e-5f, .det_eps = 1e-12f},
        "kEps=1e-3f, tol=1e-5f");
    const auto err_1e2_tol1e4 = run_batched_fp32(
        pc::SpindleM0Options<float>{.newton_eps = 1e-2f, .conv_tol = 1e-4f, .det_eps = 1e-12f},
        "kEps=1e-2f, tol=1e-4f");
    const auto err_1e2_tol1e5 = run_batched_fp32(
        pc::SpindleM0Options<float>{.newton_eps = 1e-2f, .conv_tol = 1e-5f, .det_eps = 1e-12f},
        "kEps=1e-2f, tol=1e-5f");
    const auto err_default_f32 = run_batched_fp32(
        std::nullopt, "default SpindleM0Options<float>{}");

    // In FP32, kEps=1e-3f suffers finite-difference cancellation on one handover/kink state
    // (p95 <= 0.01 Nm, isolated max ~177 Nm), whereas kEps=1e-2f eliminates the cancellation
    // and achieves <= 0.01 Nm max error across all 71 states and 9 handover states!
    require(err_1e3_tol1e4[1] < 0.01 && err_1e3_tol1e5[1] < 0.01,
            "FP32 kEps=1e-3f p95 |dTau| < 0.01 Nm");
    require(err_1e2_tol1e4[2] < 0.01, "FP32 (kEps=1e-2f, tol=1e-4f) max |dTau| < 0.01 Nm");
    require(err_1e2_tol1e5[2] < 0.01, "FP32 (kEps=1e-2f, tol=1e-5f) max |dTau| < 0.01 Nm");
    require(err_default_f32[2] < 0.01, "FP32 default SpindleM0Options<float>{} max |dTau| < 0.01 Nm");

    // Verify dense strided qM + nullptr subtreemass matches CSR on FP32 M0
    const CapturedTestState &st0 = bundle.states[bundle.states.size() / 2u];
    DataPtr d_dense(engine::make_data(m), &delete_data);
    copy_state_to_mjdata(m, d_dense.get(), st0);
    const int nv_i = static_cast<int>(m->nv);
    const int stride = nv_i + 4;
    std::vector<double> qM_full64(static_cast<std::size_t>(nv_i * nv_i), 0.0);
    mj_fullM(m, d_dense.get(), qM_full64.data());
    std::vector<float> qM_dense32(static_cast<std::size_t>(nv_i * stride), 0.0f);
    for (int r = 0; r < nv_i; ++r) {
        for (int c = 0; c < nv_i; ++c) {
            qM_dense32[to_idx(r * stride + c)] = static_cast<float>(qM_full64[to_idx(r * nv_i + c)]);
        }
    }
    const auto qpos_f32 = to_real_vec<float>(d_dense->qpos, m->nq);
    const auto qvel_f32 = to_real_vec<float>(d_dense->qvel, m->nv);
    const auto bias_f32 = to_real_vec<float>(d_dense->qfrc_bias, m->nv);
    const auto xmat_f32 = to_real_vec<float>(d_dense->xmat, m->nbody * 9);
    const auto xpos_f32 = to_real_vec<float>(d_dense->xpos, m->nbody * 3);
    const auto xanc_f32 = to_real_vec<float>(d_dense->xanchor, m->njnt * 3);
    const auto xax_f32 = to_real_vec<float>(d_dense->xaxis, m->njnt * 3);
    const auto spos_f32 = to_real_vec<float>(d_dense->site_xpos, m->nsite * 3);
    const auto scom_f32 = to_real_vec<float>(d_dense->subtree_com, m->nbody * 3);
    const auto dM_f32 = to_real_vec<float>(d_dense->M, static_cast<int>(m->nC));

    DataPtr d_csr(engine::make_data(m), &delete_data);
    copy_state_to_mjdata(m, d_csr.get(), st0);

    pc::SpindleM0State<float> st_csr{}, st_den{};
    const auto scr_csr = std::make_unique<pc::SpindleM0Scratch<float>>();
    const auto scr_den = std::make_unique<pc::SpindleM0Scratch<float>>();
    auto ctrl_csr = to_real_vec<float>(d_csr->ctrl, m->nu);
    auto ctrl_den = to_real_vec<float>(d_dense->ctrl, m->nu);
    pc::u8 ns_c = 0, dn_c = 0, ns_d = 0, dn_d = 0;

    pc::spindle_m0_begin<float, pc::GpuToolkit>(
        &params32, make_spindle_cmd<float>(st0), 0,
        qpos_f32.data(), qvel_f32.data(), bias_f32.data(),
        xmat_f32.data(), xpos_f32.data(), xanc_f32.data(), xax_f32.data(),
        spos_f32.data(), scom_f32.data(), submass_f32.data(),
        M_rowadr_i32.data(), M_rownnz_i32.data(), M_colind_i32.data(), dM_f32.data(),
        &st_csr, scr_csr.get(), ctrl_csr.data(), ns_c, dn_c);
    pc::spindle_m0_begin<float, pc::GpuToolkit>(
        &params32, make_spindle_cmd<float>(st0), 0,
        qpos_f32.data(), qvel_f32.data(), bias_f32.data(),
        xmat_f32.data(), xpos_f32.data(), xanc_f32.data(), xax_f32.data(),
        spos_f32.data(), scom_f32.data(), nullptr,
        stride, nullptr, nullptr, nullptr, qM_dense32.data(),
        &st_den, scr_den.get(), ctrl_den.data(), ns_d, dn_d);
    while (dn_c == 0 || dn_d == 0) {
        if (ns_c != 0) {
            const auto dst = model_access::mutable_buffer(d_csr->ctrl, m->nu, "c");
            for (int a = 0; a < m->nu; ++a) dst[to_idx(a)] = static_cast<double>(ctrl_csr[to_idx(a)]);
            mj_forwardSkip(m, d_csr.get(), mjSTAGE_VEL, 1);
        }
        if (ns_d != 0) {
            const auto dst = model_access::mutable_buffer(d_dense->ctrl, m->nu, "c");
            for (int a = 0; a < m->nu; ++a) dst[to_idx(a)] = static_cast<double>(ctrl_den[to_idx(a)]);
            mj_forwardSkip(m, d_dense.get(), mjSTAGE_VEL, 1);
        }
        const auto qa_c = to_real_vec<float>(d_csr->qacc, m->nv);
        const auto qa_d = to_real_vec<float>(d_dense->qacc, m->nv);
        pc::spindle_m0_step<float, pc::GpuToolkit>(
            &params32, qpos_f32.data(), qvel_f32.data(), bias_f32.data(), qa_c.data(),
            M_rowadr_i32.data(), M_rownnz_i32.data(), M_colind_i32.data(), dM_f32.data(),
            &st_csr, scr_csr.get(), ctrl_csr.data(), ns_c, dn_c,
            pc::SpindleM0Options<float>{.newton_eps = 1e-3f, .conv_tol = 1e-4f, .det_eps = 1e-12f});
        pc::spindle_m0_step<float, pc::GpuToolkit>(
            &params32, qpos_f32.data(), qvel_f32.data(), bias_f32.data(), qa_d.data(),
            stride, nullptr, nullptr, nullptr, qM_dense32.data(),
            &st_den, scr_den.get(), ctrl_den.data(), ns_d, dn_d,
            pc::SpindleM0Options<float>{.newton_eps = 1e-3f, .conv_tol = 1e-4f, .det_eps = 1e-12f});
    }
    for (int i = 0; i < pc::kNumSpindleJoints; ++i) {
        const int aid = std::span{params32.joint_actuator_id}[to_idx(i)];
        require(std::abs(ctrl_den[to_idx(aid)] - ctrl_csr[to_idx(aid)]) <= 1e-4f,
                "dense qM + nullptr submass FP32 M0 matches CSR");
    }

    // Verify 4-step ControlClock substep replay
    auto p4 = std::make_unique<pc::SpindleM0Params<float>>(params32);
    p4->control_period_steps = 4;
    pc::u8 ns_sub = 1, dn_sub = 0;
    std::vector<float> ctrl_sub(static_cast<std::size_t>(m->nu), 0.0f);
    pc::spindle_m0_begin<float, pc::GpuToolkit>(
        p4.get(), make_spindle_cmd<float>(st0), /*step_index=*/1,
        qpos_f32.data(), qvel_f32.data(), bias_f32.data(),
        xmat_f32.data(), xpos_f32.data(), xanc_f32.data(), xax_f32.data(),
        spos_f32.data(), scom_f32.data(), submass_f32.data(),
        M_rowadr_i32.data(), M_rownnz_i32.data(), M_colind_i32.data(), dM_f32.data(),
        &st_csr, scr_csr.get(), ctrl_sub.data(), ns_sub, dn_sub);
    require(dn_sub == 1 && ns_sub == 0, "4-step substep skips Newton and finishes in begin()");
    for (int i = 0; i < pc::kNumSpindleJoints; ++i) {
        const int aid = std::span{params32.joint_actuator_id}[to_idx(i)];
        require_bits_f32(ctrl_sub[to_idx(aid)], ctrl_csr[to_idx(aid)],
                         "4-step substep replays held ctrl");
    }
}

// ---- 4. Closed-loop CPU MuJoCo multi-step ride sanity check -----------

void test_closed_loop_cpu_multistep_ride() {
    std::cout << "[SP4-M0] 4. Closed-loop CPU MuJoCo multi-step ride check (480 steps: M0 vs M2K2 vs spindle_m0)...\n";

    auto boot_a = make_welded_ride_bootstrap({.track_rel = "tests/fixtures/tracks/flat.toml"});
    auto boot_b = make_welded_ride_bootstrap({.track_rel = "tests/fixtures/tracks/flat.toml"});
    auto boot_c = make_welded_ride_bootstrap({.track_rel = "tests/fixtures/tracks/flat.toml"});

    const int control_period_steps = boot_a->config.control_period_steps;
    const double crank_length_m = boot_a->config.geometry.crank_length_m;
    const rider::SpindlePose pose = rider::decode_spindle_pose(
        boot_a->config.geometry.pose, "config.geometry.pose");

    auto rt_seed = std::make_unique<runtime::NativeRideRuntime>(std::move(*boot_a));
    boot_a.reset();
    runtime::RideControl ctrl_init{};
    ctrl_init.rider_enabled = true;
    (void)rt_seed->advance(800, ctrl_init, 0.0, 0.0, std::nullopt);

    std::vector<double> rolling_state, ctrl_snap, fl_snap, tire_state_snap;
    std::vector<std::string> tire_names_snap;
    auto drive_snap = std::make_unique<drivetrain::DriveSnapshot>();
    auto rc_state_snap = std::make_unique<writers::RiderContactsState>();
    std::optional<writers::RiderContactsProbe> rc_probe_snap;
    double coast_hold_snap = 0.0;
    double delivered_crank_snap = 0.0;
    auto spindle_cfg = std::make_unique<rider::SpindleConfig>();

    bike::validation::RuntimeRigAccess::inspect(
        *rt_seed, [&](const Stepper &stepper, const runtime::PhysicalStep &pstep) {
            const mjModel *m = stepper.model();
            const mjData *d = stepper.data();
            const auto sz = engine::state_size(m, mjSTATE_INTEGRATION);
            rolling_state.resize(static_cast<std::size_t>(sz));
            engine::get_state(m, d, rolling_state.data(), mjSTATE_INTEGRATION);
            const auto ctrl_ro = model_access::readonly_buffer(d->ctrl, m->nu, "ctrl");
            ctrl_snap.assign(ctrl_ro.begin(), ctrl_ro.end());
            const auto fl_sp = model_access::readonly_buffer(m->dof_frictionloss, m->nv, "fl");
            fl_snap.assign(fl_sp.begin(), fl_sp.end());
            tire_names_snap = stepper.tire_writer().state_names();
            tire_state_snap = stepper.tire_writer().state();
            *drive_snap = stepper.drive().state();
            *rc_state_snap = stepper.rider_contacts().state();
            rc_probe_snap = stepper.rider_contacts().probe();
            coast_hold_snap = pstep.rider_control().state().coast_hold_nm.value_or(0.0);
            delivered_crank_snap = stepper.rider_contacts().state().delivered_crank_torque_nm;
            *spindle_cfg = pstep.rider_control().config();
        });
    rt_seed.reset();

    const auto wcfg_b = std::make_unique<nativecfg::WriterConfig>(nativecfg::decode(boot_b->config.writer_config));
    const auto wcfg_c = std::make_unique<nativecfg::WriterConfig>(nativecfg::decode(boot_c->config.writer_config));
    auto stepper_a = std::make_unique<Stepper>(boot_b->model.mjb, *wcfg_b);
    auto stepper_b = std::make_unique<Stepper>(boot_c->model.mjb, *wcfg_c);
    boot_b.reset();
    boot_c.reset();

    const auto init_stepper = [&](Stepper &stp) {
        const mjModel *m = stp.model();
        mjData *d = stp.data();
        std::ranges::copy(fl_snap, model_access::mutable_buffer(m->dof_frictionloss, m->nv, "fl").begin());
        engine::set_state(m, d, rolling_state.data(), mjSTATE_INTEGRATION);
        std::ranges::copy(ctrl_snap, model_access::mutable_buffer(d->ctrl, m->nu, "ctrl").begin());
        stp.tire_writer().set_state(tire_names_snap, tire_state_snap);
        stp.drive().restore(*drive_snap);
        stp.rider_contacts().assign_state(
            writers::RiderContactsState(*rc_state_snap),
            std::optional<writers::RiderContactsProbe>(rc_probe_snap));
        engine::forward(m, d);
    };
    init_stepper(*stepper_a);
    init_stepper(*stepper_b);

    mjModel *ma = stepper_a->model();
    const mjData *da = stepper_a->data();
    mjModel *mb = stepper_b->model();
    const mjData *db = stepper_b->data();

    auto ctrl_a = std::make_unique<rider::SpindleController>(ma, pose, *spindle_cfg, crank_length_m);
    auto ctrl_b = std::make_unique<rider::SpindleController>(mb, pose, *spindle_cfg, crank_length_m);
    ctrl_b->set_solver_mode(rider::SpindleSolverMode::m2k2);
    {
        auto cs = ctrl_a->state();
        cs.coast_hold_nm = coast_hold_snap;
        ctrl_a->restore(cs);
        ctrl_b->restore(cs);
    }

    auto env_spec_ptr = std::make_unique<pc::RiderJointEnvelopeSpec<double>>();
    auto &env_spec = *env_spec_ptr;
    env_spec.stiffness_nm_rad = spindle_cfg->joint_envelope_soft_k_nm_rad;
    env_spec.margin_rad = spindle_cfg->joint_envelope_soft_margin_rad;
    {
        const auto jnt_limited = model_access::readonly_buffer(ma->jnt_limited, ma->njnt, "jl");
        const auto jnt_range = model_access::readonly_buffer(ma->jnt_range, ma->njnt * 2, "jr");
        const auto qadr = model_access::readonly_buffer(ma->jnt_qposadr, ma->njnt, "qa");
        const auto dadr = model_access::readonly_buffer(ma->jnt_dofadr, ma->njnt, "da");
        for (int i = 0; i < pc::kNumSpindleJoints; ++i) {
            const int jid = mj_name2id(ma, mjOBJ_JOINT, pc::kSpindleJointNames[to_idx(i)]);
            if (jid >= 0 && jnt_limited[to_idx(jid)] != 0) {
                const int idx = env_spec.count++;
                std::span{env_spec.qpos_adrs}[to_idx(idx)] = qadr[to_idx(jid)];
                std::span{env_spec.dof_adrs}[to_idx(idx)] = dadr[to_idx(jid)];
                std::span{env_spec.lower}[to_idx(idx)] = jnt_range[to_idx(jid) * 2 + 0];
                std::span{env_spec.upper}[to_idx(idx)] = jnt_range[to_idx(jid) * 2 + 1];
            }
        }
    }

    const int root_x_jnt = mj_name2id(ma, mjOBJ_JOINT, "root_x");
    const int root_x_qpos = model_access::readonly_buffer(ma->jnt_qposadr, ma->njnt, "qa")[to_idx(root_x_jnt)];
    const int root_x_dof = model_access::readonly_buffer(ma->jnt_dofadr, ma->njnt, "da")[to_idx(root_x_jnt)];
    const double start_x = model_access::readonly_buffer(da->qpos, ma->nq, "q")[to_idx(root_x_qpos)];

    rider::RiderPosture target_posture{};
    target_posture.torso_lean_rad = 0.05;
    const double dt = ma->opt.timestep;
    constexpr int kRolloutSteps = 480;
    constexpr double kHumanTorqueNm = 30.0;

    std::vector<double> total_qfrc(static_cast<std::size_t>(ma->nv), 0.0);
    std::vector<double> tire_qfrc_buf(static_cast<std::size_t>(ma->nv), 0.0);
    std::vector<double> rc_qfrc_buf(static_cast<std::size_t>(ma->nv), 0.0);
    std::vector<double> env_qfrc_buf(static_cast<std::size_t>(ma->nv), 0.0);
    std::array<ForceComponentView, 8> susp_views{};
    std::array<ForceComponentView, 2> res_views{};
    std::vector<double> q_start(static_cast<std::size_t>(ma->nq), 0.0);
    std::vector<double> v_start(static_cast<std::size_t>(ma->nv), 0.0);

    const auto step_one = [&](Stepper &stp, rider::SpindleController &rc,
                              rider::NamedEntries<double> &held, double &prev_crank,
                              int step, double &min_pedal_n) {
        const mjModel *m = stp.model();
        mjData *d = stp.data();
        engine::forward(m, d);
        std::ranges::fill(model_access::mutable_buffer(d->qfrc_applied, m->nv, "qfrc"), 0.0);
        std::ranges::fill(model_access::mutable_buffer(d->xfrc_applied, m->nbody * 6, "xfrc"), 0.0);
        std::ranges::fill(total_qfrc, 0.0);
        const auto qvel_ro = model_access::readonly_buffer(d->qvel, m->nv, "v");
        const drivetrain::RideControl d_ctrl{
            .motor_torque_nm = std::nullopt,
            .motor_limit_nm = std::nullopt,
            .human_torque_nm = kHumanTorqueNm,
            .rider_enabled = true,
            .crank_reposition = false,
        };
        const auto ped = stp.drive().prepare(
            d_ctrl, dt, false, true, true, true, std::nullopt, std::nullopt,
            false, drivetrain::RiderShift{}, false);
        const auto susp_names = stp.suspension().component_names();
        stp.suspension().components_into(d, std::span{susp_views}.first(susp_names.size()));
        for (std::size_t i = 0; i < susp_names.size(); ++i) {
            for (int k = 0; k < m->nv; ++k) total_qfrc[to_idx(k)] += susp_views[i].values[to_idx(k)];
        }
        stp.tire_writer().compute_into(d, dt, tire_qfrc_buf);
        for (int k = 0; k < m->nv; ++k) total_qfrc[to_idx(k)] += tire_qfrc_buf[to_idx(k)];
        const bool committed = stp.tire_writer().snapshots_committed();
        const auto snaps = stp.tire_writer().snapshot_storage();
        const auto r_names = stp.resistance().component_names();
        std::array<std::vector<double>, 2> side_loads;
        std::array<std::array<bool, 64>, 2> side_work{};
        std::array<TireSideInput, 2> side_in{};
        for (std::size_t si = 0; si < 2; ++si) {
            if (committed) {
                const auto &patches = snaps[si].patches;
                const std::size_t np = std::min(patches.size(), side_work[si].size());
                side_loads[si].resize(np);
                for (std::size_t pi = 0; pi < np; ++pi) {
                    side_loads[si][pi] = patches[pi].normal_load_n;
                    side_work[si][pi] = patches[pi].working_surface;
                }
                side_in[si] = TireSideInput{
                    .patch_loads = side_loads[si],
                    .patch_working = std::span{side_work[si]}.first(np),
                    .effective_radius_m = snaps[si].effective_radius_m,
                };
            }
        }
        stp.resistance().components_into(d, side_in[0], side_in[1], std::span{res_views}.first(r_names.size()));
        for (std::size_t i = 0; i < r_names.size(); ++i) {
            for (int k = 0; k < m->nv; ++k) total_qfrc[to_idx(k)] += res_views[i].values[to_idx(k)];
        }
        stp.rider_contacts().compute_qfrc_into(d, dt, true, true, rc_qfrc_buf);
        for (int k = 0; k < m->nv; ++k) total_qfrc[to_idx(k)] += rc_qfrc_buf[to_idx(k)];
        const drivetrain::TickInputs t_in{
            .control = d_ctrl, .dt = dt,
            .speed = qvel_ro[to_idx(root_x_dof)], .sensed = prev_crank,
            .braking = false, .active = true, .advance = true, .contact = true,
            .motor_braking = false, .shift_blocked = false, .pedaling = ped,
            .slip = std::nullopt,
        };
        const auto tick = stp.drive().stage_components(t_in);
        for (const auto &comp : tick.components) {
            for (int k = 0; k < m->nv; ++k) total_qfrc[to_idx(k)] += comp.values[to_idx(k)];
        }
        stp.drive().commit(tick);
        std::ranges::fill(env_qfrc_buf, 0.0);
        double env_j = 0.0;
        pc::ErrCode env_err = pc::ErrCode::none;
        const auto env_st = pc::eval_rider_joint_envelope<double, pc::CpuExactToolkit>(
            &env_spec, static_cast<pc::i32>(m->nq), static_cast<pc::i32>(m->nv),
            d->qpos, env_qfrc_buf.data(), env_j, env_err);
        require(env_st == pc::Status::ok, "eval_rider_joint_envelope ok");
        for (int k = 0; k < m->nv; ++k) total_qfrc[to_idx(k)] += env_qfrc_buf[to_idx(k)];
        std::ranges::copy(total_qfrc, model_access::mutable_buffer(d->qfrc_applied, m->nv, "qfrc").begin());
        engine::forward(m, d);

        if (step % control_period_steps == 0) {
            rider::RiderCommand rcmd{};
            rcmd.mean_crank_torque_nm = ped.effort_nm;
            rcmd.enabled = true;
            rcmd.posture = target_posture;
            rcmd.crank_target_rate_rad_s = ped.target_rate_rad_s;
            held = rc.compute(
                d, rcmd, true, static_cast<double>(control_period_steps) * dt,
                false, std::nullopt, false, prev_crank);
        }
        rc.write(d, held);
        std::ranges::copy(model_access::readonly_buffer(d->qpos, m->nq, "q"), q_start.begin());
        std::ranges::copy(model_access::readonly_buffer(d->qvel, m->nv, "v"), v_start.begin());
        const auto prep = stp.rider_contacts().prepare_attachment_raw(d);
        engine::step(m, d);
        const auto settled = stp.rider_contacts().settle_welds(
            d, writers::RiderIntervalState{q_start, v_start}, true, &prep);
        (void)stp.drive().settle();
        prev_crank = stp.rider_contacts().state().delivered_crank_torque_nm;
        if (settled.front_pedal_reaction.has_value()) {
            min_pedal_n = std::min(min_pedal_n, settled.front_pedal_reaction->normal_n);
        }
        if (settled.rear_pedal_reaction.has_value()) {
            min_pedal_n = std::min(min_pedal_n, settled.rear_pedal_reaction->normal_n);
        }
    };

    rider::NamedEntries<double> held_a, held_b;
    double prev_a = delivered_crank_snap, prev_b = delivered_crank_snap;
    double min_ped_a = 1e9, min_ped_b = 1e9;
    for (int step = 0; step < kRolloutSteps; ++step) {
        step_one(*stepper_a, *ctrl_a, held_a, prev_a, step, min_ped_a);
        step_one(*stepper_b, *ctrl_b, held_b, prev_b, step, min_ped_b);
    }

    const double dist_a = model_access::readonly_buffer(da->qpos, ma->nq, "q")[to_idx(root_x_qpos)] - start_x;
    const double dist_b = model_access::readonly_buffer(db->qpos, mb->nq, "q")[to_idx(root_x_qpos)] - start_x;
    const double rel_err = std::abs(dist_b - dist_a) / std::max(1.0, std::abs(dist_a));
    std::cout << "  Closed-loop 480 steps: dist_M0=" << dist_a
              << " m, dist_M2K2=" << dist_b
              << " m, rel_err=" << (rel_err * 100.0) << "% | min_pedal_n M0=" << min_ped_a
              << " M2K2=" << min_ped_b << " N\n";
    require(min_ped_a >= -6.0 && min_ped_b >= -6.0, "closed-loop push-only pedal >= -6 N");
    require(rel_err <= 0.002, "closed-loop SpindleController M2K2 within <= 0.2% of M0");
}

} // namespace

int main() {
    try {
        const auto bundle = std::make_unique<MultiStateCaptureBundle>(
            capture_diverse_states_from_runtime());

        test_spindle_m0_fp64_parity(*bundle);
        test_spindle_controller_m2k2_zero_bit_parity(*bundle);
        test_spindle_m0_fp32_multiworld_and_eps(*bundle);
        test_closed_loop_cpu_multistep_ride();

        std::cout << "[SP4-M0] ALL TESTS PASSED\n";
        return 0;
    } catch (const std::exception &ex) {
        std::cerr << "[SP4-M0] FAILED: " << ex.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "[SP4-M0] FAILED: unknown exception\n";
        return 1;
    }
}
