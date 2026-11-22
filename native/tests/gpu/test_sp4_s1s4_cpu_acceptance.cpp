#include "configuration/load.hpp"
#include "configuration/model_config.hpp"
#include "configuration/physical_config.hpp"
#include "diag.hpp"
#include "golden/step_closure_bundle.hpp"
#include "gpu/cuda_kernels_abi.hpp"
#include "gpu/model_assets.hpp"
#include "gpu/runner_spindle_setup.hpp"
#include "gpu/runner_tyre_setup.hpp"
#include "gpu/sp3_autonomous_step.hpp"
#include "gpu/sp4_closed_loop_step.hpp"
#include "initialization/launch_preparation.hpp"
#include "model/rider_geometry.hpp"
#include "model_access.hpp"
#include "research/environment.hpp"
#include "rider/spindle_math.hpp"
#include "runtime/control.hpp"
#include "runtime/runtime.hpp"
#include "runtime/samples.hpp"
#include "runtime/step.hpp"
#include "stepper.hpp"
#include "terrain/trackfile.hpp"
#include "validation/runtime_access.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <mujoco/mujoco.h>
#include <numbers>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

namespace fs = std::filesystem;
namespace ga = bike::gpu::assets;
namespace ma = model_access;
namespace cfg = bike::configuration;
namespace research = bike::research;
namespace sw = runtime::sample_wire;

using ModelPtr = std::unique_ptr<mjModel, void (*)(mjModel *)>;
using DataPtr = std::unique_ptr<mjData, void (*)(mjData *)>;

constexpr double kPi = std::numbers::pi;
constexpr float kDtS = 0.00125f;

std::size_t to_index(int v) {
    return static_cast<std::size_t>(v);
}

int to_int(mjtSize v) {
    return static_cast<int>(v);
}

void require(bool cond, std::string_view msg) {
    if (!cond) {
        throw std::runtime_error(std::string(msg));
    }
}

fs::path repo_root() {
    return {BIKE_REPO_ROOT};
}

fs::path repo_file(std::string_view rel) {
    return repo_root() / fs::path(rel);
}

void write_text(const fs::path &p, std::string_view content) {
    std::ofstream ofs(p, std::ios::binary);
    require(ofs.is_open(), "failed to open file for write: " + p.string());
    ofs.write(content.data(), static_cast<std::streamsize>(content.size()));
    require(ofs.good(), "failed writing file: " + p.string());
}

void write_hfield_bin(const fs::path &p, const std::vector<double> &hf) {
    if (hf.empty()) {
        return;
    }
    std::ofstream ofs(p, std::ios::binary);
    require(ofs.is_open(), "failed to open hfield_data.bin: " + p.string());
    for (const double v : hf) {
        const auto bytes = std::bit_cast<std::array<char, sizeof(double)>>(v);
        ofs.write(bytes.data(), sizeof(double));
    }
    require(ofs.good(), "failed writing hfield_data.bin: " + p.string());
}

ModelPtr load_xml(const fs::path &xml_path) {
    std::array<char, 1024> mj_err{};
    mjModel *raw = mj_loadXML(
        xml_path.string().c_str(), nullptr, mj_err.data(), static_cast<int>(mj_err.size()));
    require(raw != nullptr, std::string("mj_loadXML failed: ") + mj_err.data());
    return {raw, &mj_deleteModel};
}

template <class DstT, class SrcT>
void copy_cast(const SrcT *src, std::size_t n, std::vector<DstT> &dst) {
    dst.resize(n);
    const auto src_span = ma::readonly_buffer(src, static_cast<mjtSize>(n), "src");
    for (std::size_t i = 0; i < n; ++i) {
        dst[i] = static_cast<DstT>(src_span[i]);
    }
}

struct CycleEnv {
    std::unique_ptr<runtime::NativeRideRuntime> runtime;
    std::int64_t period = 0;
    std::int64_t max_steps = 0;
    double finish_x_m = 0.0;
    int root_x_qpos = -1;
    int root_x_dof = -1;
};

struct CycleEnvSpec {
    std::string_view physics_rel = "examples/research/viewer_physics_welded.toml";
    std::string_view track_rel;
    double duration_s = 0.0;
    double initial_speed_mps = 0.0;
    bool stage_a_mixed_surface = false;
};

void apply_stage_a_mixed_surface_sections(bike::terrain::TrackSpec &track) {
    track.surface = "hardpack";
    track.surface_sections = {
        bike::terrain::SurfaceSection{
            .start_m = 6.0,
            .end_m = 20.0,
            .surface = "asphalt",
            .blend_m = 1.5,
        },
        bike::terrain::SurfaceSection{
            .start_m = 24.0,
            .end_m = 52.0,
            .surface = "loose",
            .blend_m = 2.0,
        },
    };
}

CycleEnv make_env(const CycleEnvSpec &env_spec) {
    auto owner = std::make_unique<research::ResearchRequest>();
    auto &request = *owner;
    request.setup.model.physics =
        cfg::load_physics(repo_file(env_spec.physics_rel));
    request.setup.model.physics->initial_speed_mps = env_spec.initial_speed_mps;
    request.setup.model.mode = "ride";
    request.setup.model.crank_joint = true;
    request.setup.model.rider = bike::model::rider_from_args("articulated_planar");
    request.setup.track = bike::terrain::load_track(repo_file(env_spec.track_rel));
    if (env_spec.stage_a_mixed_surface) {
        apply_stage_a_mixed_surface_sections(request.setup.track);
    }
    request.experiment.duration_s = env_spec.duration_s;
    request.experiment.record_decimation = 1;
    request.experiment.stop_on_model_violation = false;
    request.sensors = research::SensorConfig{};
    request.experiment.control_period_s =
        request.setup.model.physics->research.control_period_s;
    request.experiment.actuator_delay_s =
        request.setup.model.physics->research.actuator_delay_s;
    request.sensors.delivery.latency_s =
        request.setup.model.physics->research.sensor_latency_s;
    request.sensors.delivery.maximum_age_s =
        request.setup.model.physics->research.sensor_maximum_age_s;
    request.sensors.sample_period_s = 0.005;
    const double finish_x_m = request.setup.track.length_m;
    const auto prepared = research::prepare_environment(std::move(owner));
    auto bootstrap = std::make_unique<runtime::RuntimeBootstrap>(
        std::move(prepared->bootstrap.ride));
    CycleEnv env;
    env.period = prepared->bootstrap.config.control_steps;
    env.max_steps = prepared->bootstrap.config.max_steps;
    env.finish_x_m = finish_x_m;
    env.runtime = std::make_unique<runtime::NativeRideRuntime>(std::move(*bootstrap));
    bike::validation::RuntimeRigAccess::inspect(
        *env.runtime, [&](const Stepper &stepper, const runtime::PhysicalStep &) {
            const mjModel *m = stepper.model();
            const auto jnt_qposadr =
                model_access::readonly_buffer(m->jnt_qposadr, m->njnt);
            const auto jnt_dofadr =
                model_access::readonly_buffer(m->jnt_dofadr, m->njnt);
            const int rx = mj_name2id(m, mjOBJ_JOINT, "root_x");
            env.root_x_qpos = jnt_qposadr[to_index(rx)];
            env.root_x_dof = jnt_dofadr[to_index(rx)];
        });
    return env;
}

struct LegWeightProbe {
    const mjModel *model = nullptr;
    DataPtr scratch{nullptr, &mj_deleteData};
    int frame_body = -1;
    int crank_joint = -1;
    std::array<int, 2> pedal_bodies{-1, -1};
    std::array<int, 2> hip_joints{-1, -1};
    std::array<std::size_t, 2> hip_dofs{0, 0};
    std::array<std::size_t, 2> knee_dofs{0, 0};
    std::array<std::array<int, 3>, 2> leg_bodies{};
    std::array<spindle::LegLoopGeometry, 2> geom{};

    explicit LegWeightProbe(const mjModel *m)
        : model(m), scratch(mj_makeData(m), &mj_deleteData),
          frame_body(mj_name2id(m, mjOBJ_BODY, "frame")),
          crank_joint(mj_name2id(m, mjOBJ_JOINT, "crank_spin")) {
        mj_resetData(m, scratch.get());
        mj_kinematics(m, scratch.get());
        const auto jnt_dofadr = model_access::readonly_buffer(m->jnt_dofadr, m->njnt);
        const auto xanchor = model_access::readonly_buffer(scratch->xanchor, 3 * m->njnt);
        const auto xpos = model_access::readonly_buffer(scratch->xpos, 3 * m->nbody);
        for (std::size_t s = 0; s < 2; ++s) {
            const char *side = s == 0 ? "front" : "rear";
            pedal_bodies[s] = mj_name2id(m, mjOBJ_BODY, (std::string("pedal_") + side).c_str());
            hip_joints[s] = mj_name2id(m, mjOBJ_JOINT, (std::string("rider_hip_") + side).c_str());
            const int knee_j = mj_name2id(m, mjOBJ_JOINT, (std::string("rider_knee_") + side).c_str());
            hip_dofs[s] = static_cast<std::size_t>(jnt_dofadr[to_index(hip_joints[s])]);
            knee_dofs[s] = static_cast<std::size_t>(jnt_dofadr[to_index(knee_j)]);
            leg_bodies[s] = {
                mj_name2id(m, mjOBJ_BODY, (std::string("rider_thigh_") + side).c_str()),
                mj_name2id(m, mjOBJ_BODY, (std::string("rider_shank_") + side).c_str()),
                mj_name2id(m, mjOBJ_BODY, (std::string("rider_foot_") + side).c_str())};
            const auto hj = 3U * static_cast<std::size_t>(hip_joints[s]);
            const auto kj = 3U * static_cast<std::size_t>(knee_j);
            const auto pb = 3U * static_cast<std::size_t>(pedal_bodies[s]);
            const double thigh_m =
                std::hypot(xanchor[kj] - xanchor[hj], xanchor[kj + 2] - xanchor[hj + 2]);
            const double link_m =
                std::hypot(xpos[pb] - xanchor[kj], xpos[pb + 2] - xanchor[kj + 2]);
            geom[s] = {
                .thigh_m = thigh_m,
                .link_m = link_m,
                .knee_sign = 1.0,
                .thigh_angle0_rad = 0.0,
                .link_angle0_rad = 0.0,
                .side_sign = (s == 0) ? 1.0 : -1.0,
            };
        }
    }

    std::array<double, 2> weight_at_pedal_n(std::span<const double> qpos, double &out_phase_rad) {
        const auto scratch_qpos = model_access::mutable_buffer(scratch->qpos, model->nq);
        std::ranges::copy(qpos, scratch_qpos.begin());
        mj_kinematics(model, scratch.get());
        mj_comPos(model, scratch.get());
        const auto xmat = model_access::readonly_buffer(scratch->xmat, 9 * model->nbody);
        const auto xanchor = model_access::readonly_buffer(scratch->xanchor, 3 * model->njnt);
        const auto xpos = model_access::readonly_buffer(scratch->xpos, 3 * model->nbody);
        const auto fb = 9U * static_cast<std::size_t>(frame_body);
        const auto cj = 3U * static_cast<std::size_t>(crank_joint);
        const auto to_frame_xz = [&](std::span<const double> buf, std::size_t base) -> spindle::Vec2 {
            const double dx = buf[base] - xanchor[cj];
            const double dy = buf[base + 1] - xanchor[cj + 1];
            const double dz = buf[base + 2] - xanchor[cj + 2];
            return {xmat[fb] * dx + xmat[fb + 3] * dy + xmat[fb + 6] * dz,
                    xmat[fb + 2] * dx + xmat[fb + 5] * dy + xmat[fb + 8] * dz};
        };
        std::array<spindle::Vec2, 2> spindles{};
        std::array<spindle::Vec2, 2> gammas{};
        for (std::size_t s = 0; s < 2; ++s) {
            spindles[s] = to_frame_xz(xpos, 3U * static_cast<std::size_t>(pedal_bodies[s]));
            const auto hip = to_frame_xz(xanchor, 3U * static_cast<std::size_t>(hip_joints[s]));
            gammas[s] = spindle::leg_vertical_gamma(geom[s], hip, spindles[s]);
        }
        out_phase_rad = std::atan2(-spindles[0][1], spindles[0][0]);
        const auto width = static_cast<std::size_t>(model->nv);
        const auto gravity = model_access::readonly_buffer(std::data(model->opt.gravity), 3);
        const auto body_mass = model_access::readonly_buffer(model->body_mass, model->nbody);
        std::vector<mjtNum> jac(3U * width);
        std::array<double, 2> out{0.0, 0.0};
        for (std::size_t s = 0; s < 2; ++s) {
            double g_hip = 0.0;
            double g_knee = 0.0;
            for (const int b : leg_bodies[s]) {
                mj_jacBodyCom(model, scratch.get(), jac.data(), nullptr, b);
                const double m = body_mass[static_cast<std::size_t>(b)];
                g_hip -= m * (gravity[0] * jac[hip_dofs[s]] +
                              gravity[1] * jac[width + hip_dofs[s]] +
                              gravity[2] * jac[2U * width + hip_dofs[s]]);
                g_knee -= m * (gravity[0] * jac[knee_dofs[s]] +
                               gravity[1] * jac[width + knee_dofs[s]] +
                               gravity[2] * jac[2U * width + knee_dofs[s]]);
            }
            out[s] = gammas[s][0] * g_hip + gammas[s][1] * g_knee;
        }
        return out;
    }
};

struct Scenario {
    const char *id = nullptr;
    const char *physics_rel = "examples/research/viewer_physics_welded.toml";
    const char *track = nullptr;
    double duration = 0.0;
    double v0 = 0.0;
    bike::physics_core::SurfaceLabel surface_label = bike::physics_core::SurfaceLabel::hardpack;
    bike::physics_core::TyreBackendKind tyre_backend =
        bike::physics_core::TyreBackendKind::compliant_2d;
    int station_count = 0;
    double max_rel_dist_err = 0.002;
    bool diagnostic_rough = false;
    bool stage_a_mixed_surface = false;
    runtime::RideControl control{};
};

struct RunMetrics {
    std::string first_failure = "none";
    bool crashed = false;
    bool balance_lost = false;
    bool energy_contract_ok = true;
    double max_rel_energy_residual = 0.0;
    double min_press_front = std::numeric_limits<double>::infinity();
    double min_press_rear = std::numeric_limits<double>::infinity();
    double max_rising_front = -std::numeric_limits<double>::infinity();
    double max_rising_rear = -std::numeric_limits<double>::infinity();
    double min_saddle = std::numeric_limits<double>::infinity();
    double max_saddle = -std::numeric_limits<double>::infinity();
    double distance = 0.0;
    double end_speed = 0.0;
    std::vector<double> root_x_trace;
};

double percentile_sorted(const std::vector<double> &sorted_vals, double pct) {
    if (sorted_vals.empty()) {
        return 0.0;
    }
    const double pos = (pct / 100.0) * static_cast<double>(sorted_vals.size() - 1U);
    const auto lo = static_cast<std::size_t>(std::floor(pos));
    const auto hi = static_cast<std::size_t>(std::ceil(pos));
    const double frac = pos - static_cast<double>(lo);
    return sorted_vals[lo] * (1.0 - frac) + sorted_vals[hi] * frac;
}

RunMetrics run_m0(const Scenario &sc) {
    auto env = make_env(CycleEnvSpec{
        .physics_rel = sc.physics_rel,
        .track_rel = sc.track,
        .duration_s = sc.duration,
        .initial_speed_mps = sc.v0,
        .stage_a_mixed_surface = sc.stage_a_mixed_surface,
    });
    while (!env.runtime->snapshot().outcome.has_value() &&
           env.runtime->snapshot().step < env.max_steps) {
        const auto cur = env.runtime->snapshot().step;
        const auto stride = std::min(cur + std::max<std::int64_t>(1, env.period), env.max_steps);
        const auto result = env.runtime->advance(
            stride, sc.control, 0.0, 0.0, std::nullopt);
        if (result.reason == "outcome") {
            break;
        }
        const auto &rec = env.runtime->recorded_intervals();
        if (!rec.empty() &&
            rec.back()->qpos[to_index(env.root_x_qpos)] >= env.finish_x_m - 0.5) {
            break;
        }
    }
    const auto snap = env.runtime->snapshot();
    const mjModel *model = nullptr;
    bike::validation::RuntimeRigAccess::inspect(
        *env.runtime, [&](const Stepper &stepper, const runtime::PhysicalStep &) {
            model = stepper.model();
        });
    LegWeightProbe probe(model);
    const auto &rows = env.runtime->recorded_intervals();
    RunMetrics out;
    out.first_failure = snap.first_failure.has_value() ? "violation" : "none";
    out.crashed = snap.outcome.has_value() && snap.outcome->starts_with("crash:");
    if (rows.empty()) {
        return out;
    }
    out.root_x_trace.reserve(rows.size());
    const double x0 = rows.front()->qpos[to_index(env.root_x_qpos)];
    for (const auto &row : rows) {
        const auto &bal = sw::object(sw::required(row->channels, "rider_balance"));
        if (sw::boolean(sw::required(bal, "balance_lost"))) {
            out.balance_lost = true;
        }
        const auto &welds = sw::object(sw::required(row->channels, "rider_welds"));
        const double f_press = sw::number(sw::required(welds, "front_pedal_press_n"));
        const double r_press = sw::number(sw::required(welds, "rear_pedal_press_n"));
        const double s_press = sw::number(sw::required(welds, "saddle_support_n"));
        out.min_saddle = std::min(out.min_saddle, s_press);
        out.max_saddle = std::max(out.max_saddle, s_press);
        out.min_press_front = std::min(out.min_press_front, f_press);
        out.min_press_rear = std::min(out.min_press_rear, r_press);

        double phase_rad = 0.0;
        const auto weights = probe.weight_at_pedal_n(row->qpos, phase_rad);
        if (row->time_s > 2.0) {
            for (std::size_t s = 0; s < 2; ++s) {
                const double press = (s == 0) ? f_press : r_press;
                const double raw_deg = (phase_rad + (s == 0 ? 0.0 : kPi)) * 180.0 / kPi;
                const double theta_deg = std::fmod(std::fmod(raw_deg, 360.0) + 360.0, 360.0);
                if (theta_deg >= 100.0 && theta_deg <= 240.0) {
                    const double excess = press - weights[s];
                    if (s == 0) {
                        out.max_rising_front = std::max(out.max_rising_front, excess);
                    } else {
                        out.max_rising_rear = std::max(out.max_rising_rear, excess);
                    }
                }
            }
        }
        out.end_speed = row->qvel[to_index(env.root_x_dof)];
        out.distance = row->qpos[to_index(env.root_x_qpos)] - x0;
        out.root_x_trace.push_back(row->qpos[to_index(env.root_x_qpos)]);
    }
    return out;
}

bike::physics_core::SurfaceLabel surface_label_from_name(std::string_view name) {
    if (name == "asphalt") return bike::physics_core::SurfaceLabel::asphalt;
    if (name == "hardpack") return bike::physics_core::SurfaceLabel::hardpack;
    if (name == "loose") return bike::physics_core::SurfaceLabel::loose;
    if (name == "wet") return bike::physics_core::SurfaceLabel::wet;
    if (name == "scree") return bike::physics_core::SurfaceLabel::scree;
    throw std::invalid_argument("unknown surface label: " + std::string(name));
}

nativecfg::SurfaceSpec to_native_surface_spec(const bike::terrain::SurfaceSpec &s) {
    return nativecfg::SurfaceSpec{
        .name = s.name,
        .mu_peak = s.mu_peak,
        .mu_slide = s.mu_slide,
        .slip_stiffness_per_load = s.slip_stiffness_per_load,
        .stribeck_speed_mps = s.stribeck_speed_mps,
        .crr_delta = s.crr_delta,
        .load_sensitivity = s.load_sensitivity,
    };
}

nativecfg::PneumaticTyreSpec to_native_pneumatic_spec(const cfg::TyreSpecs &src) {
    return nativecfg::PneumaticTyreSpec{
        .name = src.name,
        .outer_radius_mm = src.outer_radius_mm,
        .rim_radius_mm = src.rim_radius_mm,
        .section_height_mm = src.section_height_mm,
        .pressure_bar = src.pressure_bar,
        .crr_reference = src.crr_reference,
        .casing_width_mm = src.casing_width_mm,
        .compressed_casing_mm = src.compressed_casing_mm,
        .area_factor = src.area_factor,
        .carcass_stiffness_n_mm2 = src.carcass_stiffness_n_mm2,
        .contact_length_factor = src.contact_length_factor,
        .rate_stiffening = src.rate_stiffening,
        .rate_relaxation_s = src.rate_relaxation_s,
        .loss_factor = src.loss_factor,
        .hysteresis_rate_eps_mps = src.hysteresis_rate_eps_mps,
        .relaxation_length_mm = src.relaxation_length_mm,
        .rim_stiffness_n_mm2 = src.rim_stiffness_n_mm2,
        .tread_loss_crr = src.tread_loss_crr,
    };
}

nativecfg::TireParams to_native_tire_params(const cfg::TireParameters &src) {
    nativecfg::TireParams out{};
    out.tangent_k_n_m = src.tangent_k_n_m;
    out.mu = src.mu;
    out.relaxation_length_m = src.relaxation_length_m;
    if (const auto *lin = std::get_if<cfg::TireSpec>(&src.material)) {
        out.material = nativecfg::TireMaterial{
            .radial_k_n_m = lin->radial_k_n_m,
            .loss_factor = lin->loss_factor,
            .hysteresis_rate_eps_mps = lin->hysteresis_rate_eps_mps,
            .pressure_pa_gauge = lin->pressure_pa_gauge,
            .provenance = lin->provenance,
            .valid_load_range_n = lin->valid_load_range_n,
            .table = std::nullopt,
        };
    } else if (const auto *tab = std::get_if<cfg::TabulatedTireSpec>(&src.material)) {
        out.material = nativecfg::TireMaterial{
            .radial_k_n_m = 0.0,
            .loss_factor = tab->loss_factor,
            .hysteresis_rate_eps_mps = tab->hysteresis_rate_eps_mps,
            .pressure_pa_gauge = tab->pressure_pa_gauge,
            .provenance = tab->provenance,
            .valid_load_range_n = tab->valid_load_range_n,
            .table = nativecfg::TireTable{
                .deflection_m = tab->deflection_m,
                .force_n = tab->force_n,
            },
        };
    }
    return out;
}

nativecfg::HingeDensity to_native_hinge_density(const cfg::HingeDensity &src) {
    return nativecfg::HingeDensity{
        .knots_m = src.knots_m,
        .stiffness_n_m = src.stiffness_n_m,
        .damping_ns_m = src.damping_ns_m,
        .loss_factor = src.loss_factor,
        .hysteresis_rate_eps_mps = src.hysteresis_rate_eps_mps,
        .provenance = src.provenance,
    };
}

nativecfg::DistributedMaterial to_native_distributed_material(
    const cfg::DistributedWheelMaterial &src
) {
    return nativecfg::DistributedMaterial{
        .density = to_native_hinge_density(src.density),
        .fitting_dataset_id = src.fitting_dataset_id,
        .valid_load_range_n = src.valid_load_range_n,
        .valid_deflection_range_m = src.valid_deflection_range_m,
        .calibration_status = src.calibration_status,
    };
}

struct TerrainAndTyre {
    bike::physics_core::GpuTyreModel tyre{};
    bike::physics_core::GpuTerrainTableDesc terrain_desc{};
    std::vector<float> terrain_f32;
    std::vector<bike::physics_core::i32> terrain_i32;
};

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
[[gnu::noinline]] std::unique_ptr<TerrainAndTyre> build_tyre_terrain(
    const mjModel *m,
    const mjData *d,
    const bike::gpu::GpuModelIndices &idx,
    const cfg::SimulationPhysicsConfig &physics,
    const bike::terrain::TrackSpec &track
) {
    nativecfg::TireConfig tcfg{};
    tcfg.backend = physics.tires.backend;
    tcfg.surface_mode = physics.tires.surface_mode;
    tcfg.front = to_native_tire_params(physics.tires.front);
    tcfg.rear = to_native_tire_params(physics.tires.rear);
    tcfg.significant_delta_m = physics.tires.significant_delta_m;
    tcfg.significance_fraction = physics.tires.significance_fraction;
    tcfg.distinct_normal_deg = physics.tires.distinct_normal_deg;

    if (tcfg.surface_mode == "track") {
        const auto tmap = track.surface_map();
        nativecfg::SurfaceMap smap{
            .surface = to_native_surface_spec(bike::terrain::get_surface(track.surface)),
            .sections = {},
        };
        for (const auto &sec : tmap.sections()) {
            smap.sections.push_back(nativecfg::SurfaceSection{
                .start_m = sec.start_m,
                .end_m = sec.end_m,
                .surface = to_native_surface_spec(bike::terrain::get_surface(sec.surface)),
                .blend_m = sec.blend_m,
            });
        }
        tcfg.surface_map = std::move(smap);
    }

    if (tcfg.backend == "distributed_2d_reference") {
        tcfg.distributed = nativecfg::DistributedConfig{
            .station_count = physics.tires.distributed.station_count,
            .density = to_native_hinge_density(physics.tires.distributed.density),
            .front = to_native_distributed_material(physics.tires.distributed.for_wheel("front")),
            .rear = to_native_distributed_material(physics.tires.distributed.for_wheel("rear")),
        };
    } else if (tcfg.backend == "pneumatic") {
        const bool detailed = (physics.tires.pneumatic.tier == "detailed");
        std::optional<nativecfg::SurfaceSpec> pneu_surf;
        if (tcfg.surface_mode != "track" && physics.tires.pneumatic.surface.has_value()) {
            pneu_surf = to_native_surface_spec(
                bike::terrain::get_surface(*physics.tires.pneumatic.surface));
        }
        tcfg.pneumatic = nativecfg::PneumaticConfig{
            .tier = physics.tires.pneumatic.tier,
            .n_rays = detailed ? 256 : 64,
            .half_angle_deg = 75.0,
            .discretised_brush = detailed,
            .timestep_s = physics.timestep_s,
            .surface = std::move(pneu_surf),
            .front = to_native_pneumatic_spec(physics.tires.pneumatic.front),
            .rear = to_native_pneumatic_spec(physics.tires.pneumatic.rear),
            .profile = {},
        };
    }

    const int front_geom = bike::gpu::find_wheel_contact_geom(
        m, idx.front_wheel_body, "front_contact", "geom_front_contact");
    const int rear_geom = bike::gpu::find_wheel_contact_geom(
        m, idx.rear_wheel_body, "rear_contact", "geom_rear_contact");
    require(front_geom >= 0 && rear_geom >= 0, "failed to find wheel contact geoms");

    const auto bundle = std::make_unique<bike::physics_core::HostTyreUploadBundle>(
        bike::physics_core::build_gpu_tyre_model(
            m, d, tcfg, front_geom, rear_geom,
            idx.front_wheel_body, idx.rear_wheel_body, idx.frame_body));
    const bike::gpu::HostPackedTerrain packed = bike::gpu::pack_upload_bundle_terrain(*bundle);

    auto out = std::make_unique<TerrainAndTyre>();
    out->tyre = bundle->gpu_model;
    out->terrain_desc = packed.desc;
    out->terrain_f32 = packed.f32;
    out->terrain_i32 = packed.i32;
    if (out->terrain_i32.empty()) {
        out->terrain_i32.push_back(0);
    }
    return out;
}
NATIVE_DIAG_POP

std::unique_ptr<bike::gpu::GpuSp3Config> build_sp3_config(
    const mjModel *m,
    const bike::gpu::GpuModelIndices &idx,
    const Scenario &sc,
    const cfg::SimulationPhysicsConfig &physics,
    const bike::terrain::TrackSpec &track
) {
    auto cfg_ptr = std::make_unique<bike::gpu::GpuSp3Config>();
    auto &c = *cfg_ptr;
    bike::gpu::gpu_init_default_sp3_config(c);
    c.suspension = bike::gpu::gpu_make_backend_default_suspension_params(sc.tyre_backend);
    c.default_surface_label = surface_label_from_name(track.surface);
    c.surface_section_count = 0;
    const auto ssecs = std::span{c.surface_sections};
    for (const auto &sec : track.surface_sections) {
        if (c.surface_section_count >= bike::physics_core::kMaxSurfaceSections) {
            break;
        }
        ssecs[to_index(c.surface_section_count)] = bike::physics_core::SurfaceSectionCore<float>{
            .start_m = static_cast<float>(sec.start_m),
            .end_m = static_cast<float>(sec.end_m),
            .label = surface_label_from_name(sec.surface),
        };
        c.surface_section_count += 1;
    }
    c.resistance.crr = static_cast<float>(physics.resistance.crr);
    c.resistance.rolling_taper_rad_s = static_cast<float>(physics.resistance.rolling_taper_rad_s);
    c.resistance.rho_kg_m3 = static_cast<float>(physics.resistance.rho_kg_m3);
    c.resistance.cda_m2 = static_cast<float>(physics.resistance.cda_m2);
    c.track_energy_ledger = 1;
    c.apply_direct_crank_torque = 0;
    c.arm_standstill_start = (sc.v0 <= 0.0) ? 1 : 0;
    c.seed_lean_from_launch = (sc.v0 <= 0.0) ? 1 : 0;
    if (sc.control.human_torque_nm.has_value()) {
        c.caller_human_torque_nm = static_cast<float>(*sc.control.human_torque_nm);
    }
    if (sc.control.motor_torque_nm.has_value()) {
        c.caller_motor_torque_nm = static_cast<float>(*sc.control.motor_torque_nm);
    }

    const auto jnt_qposadr = ma::readonly_buffer(m->jnt_qposadr, m->njnt, "jnt_qposadr");
    const auto jnt_dofadr = ma::readonly_buffer(m->jnt_dofadr, m->njnt, "jnt_dofadr");
    const auto jnt_limited = ma::readonly_buffer(m->jnt_limited, m->njnt, "jnt_limited");
    const auto jnt_range = ma::readonly_buffer(m->jnt_range, 2 * m->njnt, "jnt_range");
    const auto sensor_adr = ma::readonly_buffer(m->sensor_adr, m->nsensor, "sensor_adr");

    // Populate rider joint soft-edge envelope directly from mjModel jnt_range + physics config
    c.rider.envelope.stiffness_nm_rad =
        static_cast<float>(physics.articulated.joint_envelope_soft_k_nm_rad);
    c.rider.envelope.margin_rad =
        static_cast<float>(physics.articulated.joint_envelope_soft_margin_rad);
    const std::span env_qpos{c.rider.envelope.qpos_adrs};
    const std::span env_dof{c.rider.envelope.dof_adrs};
    const std::span env_lo{c.rider.envelope.lower};
    const std::span env_hi{c.rider.envelope.upper};
    const std::span idx_rqpos{idx.rider_qpos};
    int env_count = 0;
    for (int rj = 0; rj < bike::gpu::kNumRiderJoints; ++rj) {
        const int qp_adr = idx_rqpos[to_index(rj)];
        for (int j = 0; j < m->njnt; ++j) {
            if (jnt_qposadr[to_index(j)] == qp_adr && jnt_limited[to_index(j)] != 0) {
                const auto eu = to_index(env_count);
                env_qpos[eu] = qp_adr;
                env_dof[eu] = jnt_dofadr[to_index(j)];
                env_lo[eu] = static_cast<float>(jnt_range[2U * to_index(j) + 0U]);
                env_hi[eu] = static_cast<float>(jnt_range[2U * to_index(j) + 1U]);
                ++env_count;
                break;
            }
        }
    }
    if (env_count > 0) {
        c.rider.envelope.count = env_count;
    }

    c.efc_scan.grip_left_eq_id = mj_name2id(m, mjOBJ_EQUALITY, "connect_grip_left");
    c.efc_scan.grip_right_eq_id = mj_name2id(m, mjOBJ_EQUALITY, "connect_grip_right");
    c.efc_scan.saddle_eq_id = mj_name2id(m, mjOBJ_EQUALITY, "connect_saddle");
    c.efc_scan.foot_front_eq_id = mj_name2id(m, mjOBJ_EQUALITY, "connect_foot_front");
    c.efc_scan.foot_rear_eq_id = mj_name2id(m, mjOBJ_EQUALITY, "connect_foot_rear");
    c.efc_scan.frame_body_id = mj_name2id(m, mjOBJ_BODY, "frame");
    c.efc_scan.crank_dof = jnt_dofadr[to_index(mj_name2id(m, mjOBJ_JOINT, "crank_spin"))];
    c.efc_scan.front_wheel_dof = jnt_dofadr[to_index(mj_name2id(m, mjOBJ_JOINT, "front_wheel_spin"))];
    c.efc_scan.rear_wheel_dof = jnt_dofadr[to_index(mj_name2id(m, mjOBJ_JOINT, "rear_wheel_spin"))];
    int hbar_gid = mj_name2id(m, mjOBJ_GEOM, "geom_handlebar");
    if (hbar_gid < 0) {
        hbar_gid = mj_name2id(m, mjOBJ_GEOM, "handlebar");
    }
    c.efc_scan.handlebar_geom_id = hbar_gid;
    c.efc_scan.catch_geom_id = mj_name2id(m, mjOBJ_GEOM, "catch_plane");
    c.efc_scan.terrain_geom_count = 0;
    const int terrain_gid = mj_name2id(m, mjOBJ_GEOM, "terrain");
    if (terrain_gid >= 0) {
        std::span{c.efc_scan.terrain_geom_ids}[to_index(c.efc_scan.terrain_geom_count++)] =
            terrain_gid;
    }
    if (c.efc_scan.catch_geom_id >= 0 &&
        c.efc_scan.terrain_geom_count < bike::physics_core::kMaxTerrainGeoms) {
        std::span{c.efc_scan.terrain_geom_ids}[to_index(c.efc_scan.terrain_geom_count++)] =
            c.efc_scan.catch_geom_id;
    }
    c.efc_scan.rider_geom_count = 0;
    for (int g = 0; g < m->ngeom; ++g) {
        const char *gname = mj_id2name(m, mjOBJ_GEOM, g);
        if (gname != nullptr && std::string_view(gname).starts_with("geom_rider_") &&
            c.efc_scan.rider_geom_count < bike::physics_core::kMaxRiderCrashGeoms) {
            std::span{c.efc_scan.rider_geom_ids}[to_index(c.efc_scan.rider_geom_count++)] = g;
        }
    }

    const int motor_act = mj_name2id(m, mjOBJ_ACTUATOR, "mid_drive");
    c.motor_actuator_idx = motor_act;
    if (motor_act >= 0) {
        const auto ctrlrange =
            ma::readonly_buffer(m->actuator_ctrlrange, 2 * m->nu, "actuator_ctrlrange");
        c.motor_ctrl_min_nm = static_cast<float>(ctrlrange[2 * to_index(motor_act) + 0]);
        c.motor_ctrl_max_nm = static_cast<float>(ctrlrange[2 * to_index(motor_act) + 1]);
    }
    const int accel_id = mj_name2id(m, mjOBJ_SENSOR, "sensor_frame_accel");
    const int gyro_id = mj_name2id(m, mjOBJ_SENSOR, "sensor_frame_gyro");
    c.accel_sensor_adr = (accel_id >= 0) ? sensor_adr[to_index(accel_id)] : -1;
    c.gyro_sensor_adr = (gyro_id >= 0) ? sensor_adr[to_index(gyro_id)] : -1;

    const std::span<int> tid_span{c.shifting.cassette_tendon_ids};
    for (int k = 0; k < bike::physics_core::kCassetteCogCount; ++k) {
        const int tid = mj_name2id(m, mjOBJ_TENDON, std::format("freehub_cog_{:02d}", k).c_str());
        require(tid >= 0, "missing cassette tendon");
        tid_span[to_index(k)] = tid;
    }
    c.efc_scan.freehub_tendon_id = tid_span[0];
    c.efc_scan.crank_clutch_tendon_id = mj_name2id(m, mjOBJ_TENDON, "crank_clutch");
    return cfg_ptr;
}

[[gnu::noinline]] RunMetrics run_gpu_spec_cpu(
    const Scenario &sc,
    int target_steps = 0,
    bool use_m0 = false
) {
    const fs::path physics_path = repo_file(sc.physics_rel);
    const fs::path track_path = repo_file(sc.track);
    const fs::path strength_json = repo_file("examples/research/rider_strength_reference.json");
    const fs::path envelope_json = repo_file("examples/research/rider_joint_envelope_anatomical.json");

    auto spec = std::make_unique<bike::golden::StepClosureCaptureSpec>();
    spec->physics = cfg::load_physics(physics_path);
    spec->physics.initial_speed_mps = sc.v0;
    spec->track = bike::terrain::load_track(track_path);
    if (sc.stage_a_mixed_surface) {
        apply_stage_a_mixed_surface_sections(spec->track);
    }
    spec->duration_s = std::nullopt;
    spec->max_steps = std::size_t{1};
    spec->seed_decimal = "0";
    spec->physics_config_label = physics_path.filename().string();
    spec->track_label = sc.track;
    spec->asset_root = physics_path.parent_path();
    spec->strict = false;
    const auto bundle = bike::golden::capture_step_closure_bundle(*spec);
    require(!bundle->rows.empty(), "capture_step_closure_bundle returned empty rows");

    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    const fs::path tmp_dir =
        fs::temp_directory_path() / std::format("bike_sp4_accept_{}_{}", sc.id, stamp);
    fs::create_directories(tmp_dir);
    struct TempGuard {
        fs::path path;
        explicit TempGuard(fs::path p) noexcept : path(std::move(p)) {}
        TempGuard(const TempGuard &) = delete;
        TempGuard &operator=(const TempGuard &) = delete;
        TempGuard(TempGuard &&) = delete;
        TempGuard &operator=(TempGuard &&) = delete;
        ~TempGuard() {
            std::error_code ec;
            fs::remove_all(path, ec);
        }
    };
    const TempGuard guard{tmp_dir};

    const fs::path xml_path = tmp_dir / "model_12gears.xml";
    write_text(xml_path, bundle->model_12gears_xml);
    write_hfield_bin(tmp_dir / "hfield_data.bin", bundle->hfield_data);

    ModelPtr m = load_xml(xml_path);
    require(ga::load_hfield_bin(m.get(), xml_path.string().c_str()) == ga::AssetStatus::kOk,
            "load_hfield_bin failed");
    DataPtr d(mj_makeData(m.get()), &mj_deleteData);
    require(d != nullptr, "mj_makeData failed");

    const auto idx = ga::resolve_gpu_model_indices(m.get());
    {
        const auto &row0 = bundle->rows.front();
        require(static_cast<int>(row0.qpos.size()) == m->nq &&
                    static_cast<int>(row0.qvel.size()) == m->nv,
                "bundle state size mismatch");
        std::ranges::copy(row0.qpos, ma::mutable_buffer(d->qpos, m->nq, "qpos").begin());
        std::ranges::copy(row0.qvel, ma::mutable_buffer(d->qvel, m->nv, "qvel").begin());
        if (static_cast<int>(row0.qacc_warmstart.size()) == m->nv) {
            std::ranges::copy(row0.qacc_warmstart,
                              ma::mutable_buffer(d->qacc_warmstart, m->nv, "qacc_warmstart").begin());
        }
    }

    const auto sp_params = std::make_unique<bike::physics_core::SpindleM2K2Params<float>>();
    require(
        bike::gpu::build_runner_spindle_params(
            xml_path.string().c_str(),
            strength_json.string().c_str(),
            envelope_json.string().c_str(),
            1, sp_params.get()) == 0,
        "build_runner_spindle_params failed");
    mj_fwdPosition(m.get(), d.get());
    const auto tt = build_tyre_terrain(m.get(), d.get(), idx, spec->physics, spec->track);
    const auto cfg_sp3 = build_sp3_config(m.get(), idx, sc, spec->physics, spec->track);

    auto sp3_st = std::make_unique<bike::gpu::GpuSp3WorldState>();
    auto sp4_st = std::make_unique<bike::gpu::GpuSp4WorldState>();
    const auto m0_scratch = std::make_unique<bike::physics_core::SpindleM0Scratch<float>>();
    const auto ws = std::make_unique<bike::gpu::GpuWorldState>();

    const int nq = idx.nq;
    const int nv = idx.nv;
    const int nu = idx.nu;
    const int nbody = idx.nbody;
    const int njnt = to_int(m->njnt);
    const int nsite = to_int(m->nsite);
    const int ntendon = to_int(m->ntendon);
    const int nM = to_int(m->nM);
    const int nsd = to_int(m->nsensordata);

    std::vector<float> qpos_f, qvel_f, bias_f, qacc_f, xpos_f, xmat_f, xa_f, xx_f, site_f, sc_f, sm_f, bmass_f, xipos_f, M_f;
    std::vector<float> qfrc_f(to_index(nv), 0.0f);
    std::vector<float> ctrl_f(to_index(nu), 0.0f);
    std::vector<float> tr_f, fl_f, sd_f, eJ_f, ef_f;
    std::vector<float> ws_qacc_f(to_index(nv), 0.0f);
    std::vector<int> et_i, eid_i, m_ra, m_rn, m_ci;
    std::vector<int> con_wid, con_geom, con_efc, con_dim;
    copy_cast(d->qpos, to_index(nq), qpos_f);
    copy_cast(m->body_subtreemass, to_index(nbody), sm_f);
    copy_cast(m->body_mass, to_index(nbody), bmass_f);
    copy_cast(m->tendon_range, to_index(2 * ntendon), tr_f);
    copy_cast(m->dof_frictionloss, to_index(nv), fl_f);
    copy_cast(m->M_rowadr, to_index(nv), m_ra);
    copy_cast(m->M_rownnz, to_index(nv), m_rn);
    copy_cast(m->M_colind, to_index(nM), m_ci);
    std::array<float, 3> grav_f{
        static_cast<float>(std::span{m->opt.gravity}[0]),
        static_cast<float>(std::span{m->opt.gravity}[1]),
        static_cast<float>(std::span{m->opt.gravity}[2]),
    };

    const float init_grade = static_cast<float>(
        bike::initialization::rider_start_grade(
            spec->track, 2.0, spec->physics.articulated.road_lookahead_m));
    if (use_m0) {
        require(
            bike_gpu_launch_sp4_m0_init(
                nullptr, 1, &idx, cfg_sp3.get(), 0, sp3_st.get(), sp4_st.get(),
                m0_scratch.get(), ws.get(),
                qpos_f.data(), init_grade, ntendon, tr_f.data(), 2 * ntendon) == 0,
            "gpu_spec m0 init failed");
    } else {
        require(
            bike_gpu_launch_sp4_init(
                nullptr, 1, &idx, cfg_sp3.get(), 0, sp3_st.get(), sp4_st.get(), ws.get(),
                qpos_f.data(), init_grade, ntendon, tr_f.data(), 2 * ntendon) == 0,
            "gpu_spec init failed");
    }
    {
        const auto m_tr = ma::mutable_buffer(m->tendon_range, 2 * m->ntendon, "tendon_range");
        for (int i = 0; i < 2 * ntendon; ++i) {
            m_tr[to_index(i)] = static_cast<mjtNum>(tr_f[to_index(i)]);
        }
    }

    const int n_steps = (target_steps > 0)
        ? target_steps
        : static_cast<int>(std::llround(sc.duration / static_cast<double>(kDtS)));
    std::vector<double> root_x_trace;
    root_x_trace.reserve(to_index(n_steps));
    std::vector<double> ws_saved(to_index(nv), 0.0);
    for (int step = 0; step < n_steps; ++step) {
        mj_fwdPosition(m.get(), d.get());
        mj_sensorPos(m.get(), d.get());
        mj_fwdVelocity(m.get(), d.get());
        mj_sensorVel(m.get(), d.get());

        copy_cast(d->qpos, to_index(nq), qpos_f);
        copy_cast(d->qvel, to_index(nv), qvel_f);
        copy_cast(d->qfrc_bias, to_index(nv), bias_f);
        copy_cast(d->xpos, to_index(nbody) * 3U, xpos_f);
        copy_cast(d->xmat, to_index(nbody) * 9U, xmat_f);
        copy_cast(d->xanchor, to_index(njnt) * 3U, xa_f);
        copy_cast(d->xaxis, to_index(njnt) * 3U, xx_f);
        copy_cast(d->site_xpos, to_index(nsite) * 3U, site_f);
        copy_cast(d->subtree_com, to_index(nbody) * 3U, sc_f);
        copy_cast(d->M, to_index(nM), M_f);
        copy_cast(m->tendon_range, to_index(2 * ntendon), tr_f);
        copy_cast(m->dof_frictionloss, to_index(nv), fl_f);

        int m0_any_needs = 0;
        if (use_m0) {
            require(
                bike_gpu_launch_sp4_m0_pre_solve_and_begin(
                    nullptr, 1, kDtS, &idx, &tt->tyre, &tt->terrain_desc,
                    tt->terrain_f32.data(), tt->terrain_i32.data(),
                    cfg_sp3.get(), 0, sp_params.get(), sp3_st.get(), sp4_st.get(),
                    m0_scratch.get(), ws.get(),
                    qpos_f.data(), qvel_f.data(), bias_f.data(), xpos_f.data(), xmat_f.data(),
                    xa_f.data(), xx_f.data(), site_f.data(), sc_f.data(), sm_f.data(),
                    0, nM, m_ra.data(), m_rn.data(), m_ci.data(), M_f.data(),
                    qfrc_f.data(), ctrl_f.data(), ntendon, tr_f.data(), 2 * ntendon,
                    fl_f.data(), nv, ws_qacc_f.data(), &m0_any_needs) == 0,
                "gpu_spec m0 begin failed");
        } else {
            require(
                bike_gpu_launch_sp4_pre_solve_and_pass0(
                    nullptr, 1, kDtS, &idx, &tt->tyre, &tt->terrain_desc,
                    tt->terrain_f32.data(), tt->terrain_i32.data(),
                    cfg_sp3.get(), 0, sp_params.get(), sp3_st.get(), sp4_st.get(), ws.get(),
                    qpos_f.data(), qvel_f.data(), bias_f.data(), xpos_f.data(), xmat_f.data(),
                    xa_f.data(), xx_f.data(), site_f.data(), sc_f.data(), sm_f.data(),
                    0, nM, m_ra.data(), m_rn.data(), m_ci.data(), M_f.data(),
                    qfrc_f.data(), ctrl_f.data(), ntendon, tr_f.data(), 2 * ntendon, fl_f.data(), nv) == 0,
                "gpu_spec pass0 failed");
        }

        {
            const auto d_qf = ma::mutable_buffer(d->qfrc_applied, m->nv, "qfrc_applied");
            const auto m_tr = ma::mutable_buffer(m->tendon_range, 2 * m->ntendon, "tendon_range");
            const auto m_fl = ma::mutable_buffer(m->dof_frictionloss, m->nv, "dof_frictionloss");
            for (int i = 0; i < nv; ++i) {
                d_qf[to_index(i)] = static_cast<mjtNum>(qfrc_f[to_index(i)]);
                m_fl[to_index(i)] = static_cast<mjtNum>(fl_f[to_index(i)]);
            }
            for (int i = 0; i < 2 * ntendon; ++i) {
                m_tr[to_index(i)] = static_cast<mjtNum>(tr_f[to_index(i)]);
            }
        }
        mj_makeConstraint(m.get(), d.get());
        mj_island(m.get(), d.get());
        mj_projectConstraint(m.get(), d.get());
        mj_referenceConstraint(m.get(), d.get());

        {
            const auto d_ws = ma::readonly_buffer(d->qacc_warmstart, m->nv, "qacc_warmstart");
            std::ranges::copy(d_ws, ws_saved.begin());
        }
        if (use_m0) {
            int pass = 0;
            while (true) {
                if (pass > 0) {
                    std::ranges::copy(
                        ws_saved,
                        ma::mutable_buffer(d->qacc_warmstart, m->nv, "qacc_warmstart").begin());
                }
                const auto d_ctrl = ma::mutable_buffer(d->ctrl, m->nu, "ctrl");
                for (int i = 0; i < nu; ++i) {
                    d_ctrl[to_index(i)] = static_cast<mjtNum>(ctrl_f[to_index(i)]);
                }
                mj_fwdActuation(m.get(), d.get());
                mj_fwdAcceleration(m.get(), d.get());
                mj_fwdConstraint(m.get(), d.get());
                if (m0_any_needs == 0 || pass >= bike::physics_core::kM0MaxPasses) {
                    break;
                }
                copy_cast(d->qacc, to_index(nv), qacc_f);
                require(
                    bike_gpu_launch_sp4_m0_step(
                        nullptr, 1, nq, nv, nu, &idx, cfg_sp3.get(), 0, sp3_st.get(),
                        sp_params.get(), sp4_st.get(), m0_scratch.get(),
                        qpos_f.data(), qvel_f.data(), bias_f.data(), qacc_f.data(),
                        0, nM, m_ra.data(), m_rn.data(), m_ci.data(), M_f.data(),
                        ctrl_f.data(), ws_qacc_f.data(), &m0_any_needs) == 0,
                    "gpu_spec m0 step failed");
                ++pass;
            }
        } else {
            for (int pass = 0; pass < 4; ++pass) {
                if (pass > 0) {
                    std::ranges::copy(
                        ws_saved,
                        ma::mutable_buffer(d->qacc_warmstart, m->nv, "qacc_warmstart").begin());
                }
                const auto d_ctrl = ma::mutable_buffer(d->ctrl, m->nu, "ctrl");
                for (int i = 0; i < nu; ++i) {
                    d_ctrl[to_index(i)] = static_cast<mjtNum>(ctrl_f[to_index(i)]);
                }
                mj_fwdActuation(m.get(), d.get());
                mj_fwdAcceleration(m.get(), d.get());
                mj_fwdConstraint(m.get(), d.get());
                copy_cast(d->qacc, to_index(nv), qacc_f);

                if (pass == 0) {
                    require(
                        bike_gpu_launch_sp4_pass1(
                            nullptr, 1, nq, nv, nu, sp_params.get(), sp4_st.get(),
                            qpos_f.data(), qvel_f.data(), bias_f.data(), qacc_f.data(),
                            0, nM, m_ra.data(), m_rn.data(), m_ci.data(), M_f.data(), ctrl_f.data()) == 0,
                        "gpu_spec pass1 failed");
                } else if (pass == 1) {
                    require(
                        bike_gpu_launch_sp4_pass2(
                            nullptr, 1, nq, nv, nu, sp_params.get(), sp4_st.get(),
                            qpos_f.data(), qvel_f.data(), bias_f.data(), qacc_f.data(),
                            0, nM, m_ra.data(), m_rn.data(), m_ci.data(), M_f.data(), ctrl_f.data()) == 0,
                        "gpu_spec pass2 failed");
                } else if (pass == 2) {
                    require(
                        bike_gpu_launch_sp4_pass3(
                            nullptr, 1, nq, nv, nu, sp_params.get(), sp4_st.get(),
                            qpos_f.data(), qvel_f.data(), bias_f.data(), qacc_f.data(),
                            0, nM, m_ra.data(), m_rn.data(), m_ci.data(), M_f.data(), ctrl_f.data()) == 0,
                        "gpu_spec pass3 failed");
                }
            }
        }

        mj_sensorAcc(m.get(), d.get());
        mj_step2(m.get(), d.get());

        copy_cast(d->qpos, to_index(nq), qpos_f);
        copy_cast(d->qvel, to_index(nv), qvel_f);
        copy_cast(d->sensordata, to_index(nsd), sd_f);
        copy_cast(d->efc_type, to_index(std::max(1, d->nefc)), et_i);
        copy_cast(d->efc_id, to_index(std::max(1, d->nefc)), eid_i);
        copy_cast(d->efc_J, to_index(std::max(1, d->nefc)) * to_index(nv), eJ_f);
        copy_cast(d->efc_force, to_index(std::max(1, d->nefc)), ef_f);

        const int ncon_val = d->ncon;
        const auto ncon_cap = to_index(std::max(1, ncon_val));
        con_wid.assign(ncon_cap, 0);
        con_geom.assign(2U * ncon_cap, -1);
        con_efc.assign(ncon_cap, -1);
        con_dim.assign(ncon_cap, 0);
        if (ncon_val > 0) {
            const auto d_con = ma::readonly_buffer(d->contact, d->ncon, "contact");
            for (int c = 0; c < ncon_val; ++c) {
                const auto cu = to_index(c);
                const auto &[g0, g1] = d_con[cu].geom;
                con_geom[2U * cu + 0U] = g0;
                con_geom[2U * cu + 1U] = g1;
                con_efc[cu] = d_con[cu].efc_address;
                con_dim[cu] = d_con[cu].dim;
            }
        }

        if (use_m0) {
            require(
                bike_gpu_launch_sp4_m0_post_solve(
                    nullptr, 1, kDtS, &idx, cfg_sp3.get(), 0, sp_params.get(),
                    sp3_st.get(), sp4_st.get(), m0_scratch.get(), qpos_f.data(), qvel_f.data(),
                    xpos_f.data(), xmat_f.data(), xa_f.data(),
                    nsd, sd_f.data(), &d->nefc, std::max(1, d->nefc), nv,
                    et_i.data(), eid_i.data(), eJ_f.data(), ef_f.data(),
                    &ncon_val, std::max(1, ncon_val), 1,
                    con_wid.data(), con_geom.data(), con_efc.data(), con_dim.data()) == 0,
                "gpu_spec m0 post_solve failed");
        } else {
            require(
                bike_gpu_launch_sp4_post_solve(
                    nullptr, 1, kDtS, &idx, cfg_sp3.get(), 0, sp_params.get(),
                    sp3_st.get(), sp4_st.get(), qpos_f.data(), qvel_f.data(),
                    xpos_f.data(), xmat_f.data(), xa_f.data(),
                    nsd, sd_f.data(), &d->nefc, std::max(1, d->nefc), nv,
                    et_i.data(), eid_i.data(), eJ_f.data(), ef_f.data(),
                    &ncon_val, std::max(1, ncon_val), 1,
                    con_wid.data(), con_geom.data(), con_efc.data(), con_dim.data()) == 0,
                "gpu_spec post_solve failed");
        }

        const auto d_qp = ma::mutable_buffer(d->qpos, m->nq, "qpos");
        for (int i = 0; i < nq; ++i) {
            require(std::isfinite(qpos_f[to_index(i)]), "nonfinite qpos during gpu_spec run");
            d_qp[to_index(i)] = static_cast<mjtNum>(qpos_f[to_index(i)]);
        }
        mj_fwdPosition(m.get(), d.get());
        copy_cast(d->xipos, to_index(nbody) * 3U, xipos_f);
        copy_cast(d->M, to_index(nM), M_f);

        const int ep_rc = bike_gpu_launch_sp4_energy_post_step(
            nullptr, 1, kDtS, &idx, &tt->tyre, &tt->terrain_desc,
            tt->terrain_f32.data(), cfg_sp3.get(), 0, sp_params.get(),
            sp3_st.get(), sp4_st.get(), ws.get(),
            0, nM, m_ra.data(), m_rn.data(), m_ci.data(), M_f.data(),
            qpos_f.data(), qvel_f.data(), ctrl_f.data(),
            bmass_f.data(), xipos_f.data(), grav_f.data(),
            &d->nefc, std::max(1, d->nefc), nv,
            et_i.data(), eJ_f.data(), ef_f.data());
        require(
            ep_rc == 0,
            std::format("gpu_spec energy_post_step failed sc={} m0={} step={} rc={}",
                        sc.id, use_m0, step, ep_rc));

        root_x_trace.push_back(static_cast<double>(d_qp[to_index(idx.root_x_qpos)]));
    }

    require(sp4_st->sample_count == n_steps, "sample_count mismatch");
    RunMetrics out;
    out.crashed =
        (sp3_st->sp3.monitors.crash.cause != bike::physics_core::CrashCause::none ||
         sp3_st->last_efc.contact_crash_cause != bike::physics_core::CrashCause::none);
    out.balance_lost = (sp3_st->sp3.monitors.balance.triggered != 0);
    out.first_failure = (out.crashed || out.balance_lost) ? "violation" : "none";
    out.energy_contract_ok = (sp4_st->energy_contract_ok != 0);
    out.max_rel_energy_residual = static_cast<double>(sp4_st->max_rel_energy_residual);
    out.min_press_front = static_cast<double>(sp4_st->min_press_front_n);
    out.min_press_rear = static_cast<double>(sp4_st->min_press_rear_n);
    out.max_rising_front = static_cast<double>(sp4_st->max_rising_front_excess_n);
    out.max_rising_rear = static_cast<double>(sp4_st->max_rising_rear_excess_n);
    out.min_saddle = static_cast<double>(sp4_st->min_saddle_force_n);
    out.max_saddle = static_cast<double>(sp4_st->max_saddle_force_n);
    out.distance = static_cast<double>(sp4_st->end_root_x_m - sp4_st->start_root_x_m);
    out.end_speed = static_cast<double>(sp4_st->end_speed_mps);
    out.root_x_trace = std::move(root_x_trace);
    return out;
}

double compute_rear_r_eff() {
    auto flat = make_env(CycleEnvSpec{
        .track_rel = "tests/fixtures/tracks/flat.toml",
        .duration_s = 1.0,
        .initial_speed_mps = 0.0,
    });
    while (!flat.runtime->snapshot().outcome.has_value() &&
           flat.runtime->snapshot().step < flat.max_steps) {
        const auto r = flat.runtime->advance(
            flat.runtime->snapshot().step + flat.period,
            runtime::RideControl{}, 0.0, 0.0, std::nullopt);
        if (r.reason == "outcome") {
            break;
        }
    }
    const auto &rows = flat.runtime->recorded_intervals();
    require(!rows.empty(), "flat settle produced no rows");
    const auto &tires = sw::object(sw::required(rows.front()->channels, "tires"));
    const auto &rear = sw::object(sw::required(tires, "rear"));
    return sw::number(sw::required(rear, "effective_radius_m"));
}

} // namespace

int main() {
    try {
        auto welded = std::make_unique<cfg::SimulationPhysicsConfig>(
            cfg::load_physics(repo_file("examples/research/viewer_physics_welded.toml")));
        const double gear_ratio =
            static_cast<double>(welded->drive.gearing.front_teeth) /
            static_cast<double>(welded->drive.gearing.rear_teeth);
        const double rear_r_eff = compute_rear_r_eff();
        const double steady90_v0 = 90.0 * 2.0 * kPi / 60.0 * gear_ratio * rear_r_eff;

        const std::array<Scenario, 10> scenarios{{
            {.id = "S1_flat",
             .physics_rel = "examples/research/viewer_physics_welded.toml",
             .track = "tests/fixtures/tracks/flat.toml",
             .duration = 6.0,
             .v0 = 0.0,
             .surface_label = bike::physics_core::SurfaceLabel::hardpack,
             .tyre_backend = bike::physics_core::TyreBackendKind::compliant_2d,
             .station_count = 0,
             .max_rel_dist_err = 0.002,
             .control = runtime::RideControl{}},
            {.id = "S2_climb15",
             .physics_rel = "examples/research/viewer_physics_welded.toml",
             .track = "tests/fixtures/tracks/climb_15pct.toml",
             .duration = 25.0,
             .v0 = 0.0,
             .surface_label = bike::physics_core::SurfaceLabel::asphalt,
             .tyre_backend = bike::physics_core::TyreBackendKind::compliant_2d,
             .station_count = 0,
             .max_rel_dist_err = 0.002,
             .control = runtime::RideControl{}},
            {.id = "S3_steady90",
             .physics_rel = "examples/research/viewer_physics_welded.toml",
             .track = "tests/fixtures/tracks/flat.toml",
             .duration = 8.0,
             .v0 = steady90_v0,
             .surface_label = bike::physics_core::SurfaceLabel::hardpack,
             .tyre_backend = bike::physics_core::TyreBackendKind::compliant_2d,
             .station_count = 0,
             .max_rel_dist_err = 0.002,
             .control = runtime::RideControl{.motor_torque_nm = std::nullopt,
                                             .motor_limit_nm = std::nullopt,
                                             .human_torque_nm = 24.0,
                                             .crank_target_rate_rad_s = std::nullopt,
                                             .posture = std::nullopt}},
            {.id = "S4_start_uphill15_60nm",
             .physics_rel = "examples/research/viewer_physics_welded.toml",
             .track = "tests/fixtures/tracks/climb_15pct.toml",
             .duration = 10.0,
             .v0 = 0.0,
             .surface_label = bike::physics_core::SurfaceLabel::asphalt,
             .tyre_backend = bike::physics_core::TyreBackendKind::compliant_2d,
             .station_count = 0,
             .max_rel_dist_err = 0.002,
             .control = runtime::RideControl{.motor_torque_nm = 0.0,
                                             .motor_limit_nm = std::nullopt,
                                             .human_torque_nm = 60.0,
                                             .crank_target_rate_rad_s = std::nullopt,
                                             .posture = std::nullopt}},
            {.id = "G1_welded_flat_30s",
             .physics_rel = "examples/research/viewer_physics_welded.toml",
             .track = "tests/fixtures/tracks/flat.toml",
             .duration = 30.0,
             .v0 = 0.0,
             .surface_label = bike::physics_core::SurfaceLabel::hardpack,
             .tyre_backend = bike::physics_core::TyreBackendKind::compliant_2d,
             .station_count = 0,
             .max_rel_dist_err = 0.0035,
             .diagnostic_rough = false,
             .control = runtime::RideControl{}},
            {.id = "G2_welded_rough_uphill_extreme_30s",
             .physics_rel = "examples/research/viewer_physics_welded.toml",
             .track = "examples/research/rough_uphill_extreme.toml",
             .duration = 30.0,
             .v0 = 0.0,
             .surface_label = bike::physics_core::SurfaceLabel::hardpack,
             .tyre_backend = bike::physics_core::TyreBackendKind::compliant_2d,
             .station_count = 0,
             .max_rel_dist_err = 0.025,
             .diagnostic_rough = true,
             .control = runtime::RideControl{}},
            {.id = "G3_distributed_flat_30s",
             .physics_rel = "examples/research/viewer_physics_distributed.toml",
             .track = "tests/fixtures/tracks/flat.toml",
             .duration = 30.0,
             .v0 = 0.0,
             .surface_label = bike::physics_core::SurfaceLabel::hardpack,
             .tyre_backend = bike::physics_core::TyreBackendKind::distributed_2d_reference,
             .station_count = 128,
             .max_rel_dist_err = 0.002,
             .diagnostic_rough = false,
             .control = runtime::RideControl{}},
            {.id = "G4_distributed_rough_uphill_extreme_30s",
             .physics_rel = "examples/research/viewer_physics_distributed.toml",
             .track = "examples/research/rough_uphill_extreme.toml",
             .duration = 30.0,
             .v0 = 0.0,
             .surface_label = bike::physics_core::SurfaceLabel::hardpack,
             .tyre_backend = bike::physics_core::TyreBackendKind::distributed_2d_reference,
             .station_count = 128,
             .max_rel_dist_err = 0.010,
             .diagnostic_rough = true,
             .control = runtime::RideControl{}},
            {.id = "G5_pneumatic_flat_30s",
             .physics_rel = "examples/research/viewer_physics_pneumatic.toml",
             .track = "tests/fixtures/tracks/flat.toml",
             .duration = 30.0,
             .v0 = 0.0,
             .surface_label = bike::physics_core::SurfaceLabel::hardpack,
             .tyre_backend = bike::physics_core::TyreBackendKind::pneumatic,
             .station_count = 64,
             .max_rel_dist_err = 0.005,
             .diagnostic_rough = false,
             .control = runtime::RideControl{}},
            {.id = "G6_pneumatic_mixed_surface",
             .physics_rel = "examples/research/viewer_physics_pneumatic.toml",
             .track = "examples/research/rough_uphill_extreme.toml",
             .duration = 15.0,
             .v0 = 0.0,
             .surface_label = bike::physics_core::SurfaceLabel::hardpack,
             .tyre_backend = bike::physics_core::TyreBackendKind::pneumatic,
             .station_count = 64,
             .max_rel_dist_err = 0.025,
             .diagnostic_rough = true,
             .stage_a_mixed_surface = true,
             .control = runtime::RideControl{}},
        }};

        bool all_ok = true;
        for (const auto &sc : scenarios) {
            const RunMetrics m0 = run_m0(sc);
            for (const bool use_m0 : {false, true}) {
                const char *solver_tag = use_m0 ? "M0" : "M2K2";
                const RunMetrics gpu =
                    run_gpu_spec_cpu(sc, static_cast<int>(m0.root_x_trace.size()), use_m0);
                const double dist_rel_err =
                    std::abs(gpu.distance - m0.distance) / std::max(1.0, std::abs(m0.distance));
                const double min_press_gpu = std::min(gpu.min_press_front, gpu.min_press_rear);
                const double max_rising_gpu = std::max(gpu.max_rising_front, gpu.max_rising_rear);

                const std::size_t n_cmp =
                    std::min(m0.root_x_trace.size(), gpu.root_x_trace.size());
                std::vector<double> dx_abs;
                dx_abs.reserve(n_cmp);
                for (std::size_t i = 0; i < n_cmp; ++i) {
                    dx_abs.push_back(std::abs(gpu.root_x_trace[i] - m0.root_x_trace[i]));
                }
                std::ranges::sort(dx_abs);
                const double dx_p50 = percentile_sorted(dx_abs, 50.0);
                const double dx_p95 = percentile_sorted(dx_abs, 95.0);
                const double dx_max = dx_abs.empty() ? 0.0 : dx_abs.back();

                std::cout << std::format(
                    "[{}:{}] steps={} CPU_M0(fail={} crash={} bal={} dist={:.4f} vend={:.4f} press={:+.2f}/{:+.2f} rise={:+.2f}/{:+.2f} saddle={:+.1f}..{:.1f}) "
                    "GPU(fail={} crash={} bal={} dist={:.4f} vend={:.4f} rel={:.4f}% |dx|=p50:{:.5f}/p95:{:.5f}/max:{:.5f}m press={:+.2f}/{:+.2f} rise={:+.2f}/{:+.2f} saddle={:+.1f}..{:.1f} energy_ok={} rel_e={:.4f})\n",
                    sc.id, solver_tag, n_cmp,
                    m0.first_failure, m0.crashed, m0.balance_lost, m0.distance, m0.end_speed,
                    m0.min_press_front, m0.min_press_rear,
                    m0.max_rising_front, m0.max_rising_rear, m0.min_saddle, m0.max_saddle,
                    gpu.first_failure, gpu.crashed, gpu.balance_lost, gpu.distance, gpu.end_speed,
                    dist_rel_err * 100.0, dx_p50, dx_p95, dx_max,
                    gpu.min_press_front, gpu.min_press_rear,
                    gpu.max_rising_front, gpu.max_rising_rear, gpu.min_saddle, gpu.max_saddle,
                    gpu.energy_contract_ok, gpu.max_rel_energy_residual);

                const bool energy_ok =
                    gpu.energy_contract_ok && (gpu.max_rel_energy_residual <= 0.05);
                const bool ok = sc.diagnostic_rough
                    ? (!m0.crashed &&
                       !gpu.crashed &&
                       gpu.balance_lost == m0.balance_lost &&
                       gpu.max_saddle <= 1750.0 &&
                       energy_ok &&
                       dist_rel_err <= sc.max_rel_dist_err)
                    : (m0.first_failure == "none" &&
                       gpu.first_failure == "none" &&
                       min_press_gpu >= -6.0 &&
                       max_rising_gpu <= 6.0 &&
                       gpu.min_saddle >= -6.0 &&
                       gpu.max_saddle <= 1750.0 &&
                       energy_ok &&
                       dist_rel_err <= sc.max_rel_dist_err);
                if (!ok) {
                    all_ok = false;
                }
            }
        }
        require(all_ok, "one or more S1-S4 / G1-G6 acceptance scenarios failed");
        std::cout << "ALL sp4_s1s4_cpu_acceptance scenarios PASSED\n";
        return 0;
    } catch (const std::exception &ex) {
        std::cerr << "FAIL: " << ex.what() << '\n';
        return 1;
    }
}
