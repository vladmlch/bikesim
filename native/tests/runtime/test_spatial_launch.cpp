#include "support/fixture_test.hpp"

#include "configuration/load.hpp"
#include "geometry/specs.hpp"
#include "initialization/capture.hpp"
#include "initialization/setup_owner.hpp"
#include "io/value.hpp"
#include "model/builder.hpp"
#include "model/mapping.hpp"
#include "model_access.hpp"
#include "rider/state_codec.hpp"
#include "runtime/accounting.hpp"
#include "runtime/bootstrap.hpp"
#include "runtime/control.hpp"
#include "runtime/launch_support.hpp"
#include "runtime/project_config.hpp"
#include "runtime/research_metrics.hpp"
#include "runtime/runtime.hpp"
#include "runtime/samples.hpp"
#include "runtime/static_brake.hpp"
#include "runtime/status.hpp"
#include "runtime/step.hpp"
#include "spatial/frames.hpp"
#include "spatial/types.hpp"
#include "stepper.hpp"
#include "terrain/extrusion3d.hpp"
#include "terrain/heightfield3d.hpp"
#include "terrain/profile.hpp"
#include "writer_config.hpp"
#include "writers/compliant_3d_tire.hpp"
#include "writers/drivetrain.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <numbers>
#include <optional>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

namespace cfg = bike::configuration;
namespace init = bike::initialization;
namespace mdl = bike::model;
namespace sp = bike::spatial;
namespace trn = bike::terrain;

std::filesystem::path repo_file(std::string_view relative) {
    return bt::fixture("requests/welded_flat.toml")
               .parent_path()
               .parent_path()
               .parent_path()
               .parent_path() /
           std::filesystem::path(relative);
}

double dot6(std::span<const double> a, std::span<const double> b) noexcept {
    double sum = 0.0;
    for (std::size_t i = 0; i < 6U; ++i) {
        sum += a[i] * b[i];
    }
    return sum;
}

sp::SpatialConfig make_standstill_spatial_config() {
    sp::SpatialConfig out{};
    out.tire_backend = sp::SpatialTireBackend::compliant_3d;
    out.tire_preset = sp::SpatialTirePreset::mtb_2p8_1p2bar;
    out.start_mode = sp::SpatialStartMode::standstill;
    out.corridor_half_width_m = 1.5;
    out.capsize_roll_rad = 1.0;
    out.steer.range_deg = 45.0;
    out.steer.control_period_s = 0.005;
    out.steer.reaction_delay_s = 0.010;
    out.steer.max_torque_nm = 35.0;
    out.start_support = sp::SpatialStartSupportSpec{
        .release_speed_mps = 1.6,
        .lateral_stiffness_n_per_m = 4000.0,
        .lateral_damping_ns_per_m = 400.0,
        .roll_stiffness_nm_per_rad = 2500.0,
        .roll_damping_nms_per_rad = 250.0,
    };
    return out;
}

std::unique_ptr<cfg::SimulationPhysicsConfig> load_articulated_launch_physics() {
    auto phys = std::make_unique<cfg::SimulationPhysicsConfig>(cfg::load_physics(
        repo_file("examples/research/viewer_physics_pneumatic.toml")));
    phys->physics_mode = "physical";
    phys->drive_mode = "articulated_effort";
    phys->tires.backend = "compliant_2d";
    phys->tires.surface_mode = "track";
    phys->initial_speed_mps = 0.0;
    phys->equilibrium_cache_enabled = false;
    phys->equilibrium_state_file.reset();
    phys->seated_climb.enabled = true;
    phys->rider_launch.plan_enabled = true;
    phys->drive.shifting.enabled = true;
    cfg::post_init(*phys);
    return phys;
}

trn::TrackSpec make_constant_grade_track(double length_m, double grade) {
    trn::TrackSpec track = trn::flat(length_m);
    if (grade != 0.0) {
        track.grade_profile =
            trn::GradeProfile{{{0.0, grade}, {length_m, grade}}};
    }
    return track;
}

[[gnu::noinline]] void test_launch_support_orthogonality_and_fade() {
    const sp::SpatialStartSupportSpec spec{
        .release_speed_mps = 1.6,
        .lateral_stiffness_n_per_m = 4000.0,
        .lateral_damping_ns_per_m = 400.0,
        .roll_stiffness_nm_per_rad = 2200.0,
        .roll_damping_nms_per_rad = 220.0,
    };
    runtime::LaunchSupport support(runtime::LaunchSupportConfig{
        .spec = spec,
        .root_free_qposadr = 0,
        .root_free_dofadr = 0,
        .standstill_speed_mps = 0.1,
        .initially_armed = true,
    });

    // Quintic C^1 fade-out curve checks.
    bt::require(std::abs(support.fade_weight(0.0) - 1.0) <= 1e-12,
                "fade_weight(0) == 1");
    bt::require(std::abs(support.fade_weight(0.5 * spec.release_speed_mps) - 1.0) <= 1e-12,
                "fade_weight(0.5*v_release) == 1");
    bt::require(std::abs(support.fade_weight(0.75 * spec.release_speed_mps) - 0.5) <= 1e-12,
                "fade_weight(midpoint) == 0.5");
    bt::require(support.fade_weight(spec.release_speed_mps) == 0.0,
                "fade_weight(v_release) == 0");

    // Non-zero attitude: roll = 0.09 rad, pitch = 0.22 rad (steep climb), yaw = 0.70 rad.
    const double phi = 0.09;
    const double theta = 0.22;
    const double psi = 0.70;
    const sp::Quat quat = sp::quat_from_euler_zyx(phi, theta, psi);

    std::array<double, 7> qpos{10.2, 0.08, 0.8, quat[0], quat[1], quat[2], quat[3]};
    std::array<double, 6> qvel{0.4, -0.2, 0.1, 0.15, -0.08, 0.12};
    std::array<double, 3> frame_pos_ref{10.2, 0.08, 0.8};
    std::array<double, 6> qfrc{};

    // Latch reference on first advancing step.
    const double w0 = support.compute_into(
        runtime::LaunchSupportInput{
            .qpos = qpos,
            .qvel = qvel,
            .frame_pos_world_m = frame_pos_ref,
            .forward_speed_mps = 0.0,
            .motor_braking = false,
            .advance = true,
        },
        qfrc);
    bt::require(w0 == 1.0 && support.armed() && support.state().reference_latched,
                "first advancing step latches reference while armed");

    // Displace laterally by +0.04 m along e_lat(psi) so both spring and damper are active.
    const double sin_psi = std::sin(psi);
    const double cos_psi = std::cos(psi);
    const std::array<double, 3> frame_pos_disp{
        frame_pos_ref[0] - 0.04 * sin_psi,
        frame_pos_ref[1] + 0.04 * cos_psi,
        frame_pos_ref[2] + 0.05};
    const double w1 = support.compute_into(
        runtime::LaunchSupportInput{
            .qpos = qpos,
            .qvel = qvel,
            .frame_pos_world_m = frame_pos_disp,
            .forward_speed_mps = 0.4,
            .motor_braking = false,
            .advance = true,
        },
        qfrc);
    bt::require(w1 == 1.0, "fade weight remains 1 below 0.5*v_release");

    // Verify exact orthogonality against virtual forward, +Z heave, pitch rate, yaw rate.
    const std::array<double, 6> dq_fwd{cos_psi, sin_psi, 0.0, 0.0, 0.0, 0.0};
    const std::array<double, 6> dq_heave{0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    const double sin_phi = std::sin(phi);
    const double cos_phi = std::cos(phi);
    const double sin_theta = std::sin(theta);
    const double cos_theta = std::cos(theta);
    const std::array<double, 6> dq_pitch{0.0, 0.0, 0.0, 0.0, cos_phi, -sin_phi};
    const std::array<double, 6> dq_yaw{
        0.0, 0.0, 0.0, -sin_theta, sin_phi * cos_theta, cos_phi * cos_theta};

    bt::require(std::abs(dot6(qfrc, dq_fwd)) <= 1e-12,
                "LaunchSupport does zero virtual work on forward horizontal motion");
    bt::require(std::abs(dot6(qfrc, dq_heave)) <= 1e-12,
                "LaunchSupport does zero virtual work on +Z vertical heave");
    bt::require(std::abs(dot6(qfrc, dq_pitch)) <= 1e-12,
                "LaunchSupport does zero virtual work on ZYX pitch rate");
    bt::require(std::abs(dot6(qfrc, dq_yaw)) <= 1e-12,
                "LaunchSupport does zero virtual work on ZYX yaw rate");

    // Probe (advance=false) must NOT disarm even at v_fwd >= v_release.
    const double w_probe = support.compute_into(
        runtime::LaunchSupportInput{
            .qpos = qpos,
            .qvel = qvel,
            .frame_pos_world_m = frame_pos_disp,
            .forward_speed_mps = spec.release_speed_mps + 0.5,
            .motor_braking = false,
            .advance = false,
        },
        qfrc);
    bt::require(w_probe == 0.0 && support.armed(),
                "probe at v >= v_release evaluates 0 force without disarming");

    // Advancing past v_release disarms.
    const double w_disarm = support.compute_into(
        runtime::LaunchSupportInput{
            .qpos = qpos,
            .qvel = qvel,
            .frame_pos_world_m = frame_pos_disp,
            .forward_speed_mps = spec.release_speed_mps,
            .motor_braking = false,
            .advance = true,
        },
        qfrc);
    bt::require(w_disarm == 0.0 && !support.armed(),
                "advancing past v_release disarms LaunchSupport");

    const auto disarmed_state = support.state();

    // Standstill WITHOUT brakes must NOT re-arm; moving WITH brakes must NOT re-arm.
    (void)support.compute_into(
        runtime::LaunchSupportInput{
            .qpos = qpos,
            .qvel = qvel,
            .frame_pos_world_m = frame_pos_disp,
            .forward_speed_mps = 0.02,
            .motor_braking = false,
            .advance = true,
        },
        qfrc);
    bt::require(!support.armed(), "standstill without motor_braking stays disarmed");
    (void)support.compute_into(
        runtime::LaunchSupportInput{
            .qpos = qpos,
            .qvel = qvel,
            .frame_pos_world_m = frame_pos_disp,
            .forward_speed_mps = 0.4,
            .motor_braking = true,
            .advance = true,
        },
        qfrc);
    bt::require(!support.armed(), "moving with motor_braking stays disarmed");

    // Probe while disarmed at standstill+brake must NOT re-arm.
    (void)support.compute_into(
        runtime::LaunchSupportInput{
            .qpos = qpos,
            .qvel = qvel,
            .frame_pos_world_m = frame_pos_disp,
            .forward_speed_mps = 0.02,
            .motor_braking = true,
            .advance = false,
        },
        qfrc);
    bt::require(!support.armed(), "probe standstill+braking does not re-arm");

    // Advancing standstill + motor_braking re-arms and latches new reference.
    const std::array<double, 3> new_ref{25.0, -0.4, 1.2};
    const double w_rearm = support.compute_into(
        runtime::LaunchSupportInput{
            .qpos = qpos,
            .qvel = qvel,
            .frame_pos_world_m = new_ref,
            .forward_speed_mps = 0.02,
            .motor_braking = true,
            .advance = true,
        },
        qfrc);
    bt::require(w_rearm == 1.0 && support.armed() &&
                    support.state().reference_latched &&
                    support.state().ref_pos_m == new_ref,
                "advancing standstill+motor_braking re-arms and latches new ref");

    // State round-trip restore returns to disarmed snapshot.
    support.restore(disarmed_state);
    bt::require(!support.armed(), "restore() restores disarmed state");
}

[[gnu::noinline]] std::unique_ptr<init::SetupOwner>
make_owner_from_ptr(std::unique_ptr<init::SetupRequest> req) {
    return init::make_setup(std::move(*req));
}

[[gnu::noinline]] void check_probe_components(
    runtime::PhysicalStep &phys_step, const runtime::RideControl &control,
    bool expect_launch_support, std::string_view label) {
    const auto probe = phys_step.probe_step_inputs(control, 0.0, 0.0);
    const bool has_steer_motor = std::ranges::any_of(
        probe.components,
        [](const auto &c) { return c.first == "act_rider_steer"; });
    bt::require(!has_steer_motor,
                std::string(label) + ": pure hand steering omits act_rider_steer motor");
    const bool has_launch_support_comp = std::ranges::any_of(
        probe.components,
        [](const auto &c) { return c.first == "launch_support"; });
    bt::require(has_launch_support_comp == expect_launch_support,
                std::string(label) +
                    (expect_launch_support ? ": standstill LaunchSupport active"
                                           : ": LaunchSupport disarmed"));
}

[[gnu::noinline]] std::unique_ptr<init::SetupRequest>
load_planar_welded_request() {
    auto req = std::make_unique<init::SetupRequest>(
        init::read_setup_request(bt::fixture("requests/welded_flat.toml")));
    if (!req->model.physics.has_value()) {
        throw std::runtime_error("welded_flat missing physics config");
    }
    auto &phys = *req->model.physics;
    phys.equilibrium_cache_enabled = false;
    phys.equilibrium_state_file.reset();
    req->initial_state.reset();
    req->runtime.record_retention = runtime::RetentionMode::full;
    return req;
}

[[gnu::noinline]] void test_planar_0bit_shield() {
    auto owner = make_owner_from_ptr(load_planar_welded_request());
    const auto eq = init::settle(*owner);
    bt::require(eq.residual_qacc <= 0.05, "planar welded_flat settles cleanly");
    auto bootstrap = std::make_unique<runtime::RuntimeBootstrap>(
        owner->seal_bootstrap());
    auto rt = std::make_unique<runtime::NativeRideRuntime>(std::move(*bootstrap));
    bootstrap.reset();
    runtime::RideControl control{};
    control.rider_enabled = true;
    (void)rt->advance(200, control, 0.0, 0.0, std::nullopt);
    const auto intervals = rt->recorded_intervals();
    bt::require(!intervals.empty(), "planar run produces recorded intervals");
    for (const auto &sample : intervals) {
        const auto *ew = runtime::sample_wire::find(sample->channels, "energy");
        if (ew == nullptr) {
            continue;
        }
        const auto &eobj = runtime::sample_wire::object(*ew);
        bt::require(runtime::sample_wire::find(eobj, "launch_support_work_j") == nullptr &&
                        runtime::sample_wire::find(eobj, "launch_support_loss_j") == nullptr,
                    "planar energy wire omits launch_support columns (0-bit shield)");
    }
    rt->close();
}

[[gnu::noinline]] void test_3d_heading_lookahead_and_steer_guard() {
    const double track_len = 80.0;
    const double grade = 0.15;
    const trn::TrackSpec track = make_constant_grade_track(track_len, grade);
    auto req = std::make_unique<mdl::BuildRequest>();
    req->mode = "ride";
    req->crank_joint = true;
    req->rider = mdl::rider_from_args("articulated_3d");
    req->physics = *load_articulated_launch_physics();
    req->spatial = make_standstill_spatial_config();
    const double ground_z_m = bike::geometry::compute_ground_z(req->bike) / 1000.0;
    const trn::ExtrudedTrack3d terrain3d(
        track, req->spatial->corridor_half_width_m, ground_z_m);
    const auto gs = trn::make_grid3d_spec(
        terrain3d.bounds(), trn::DEFAULT_RESOLUTION_M, 0.25);
    const auto hfield_grid = trn::sample_to_grid3d(terrain3d, gs);
    const auto hf = trn::build_heightfield_3d(hfield_grid, ground_z_m);
    req->field = hf.spec;

    const std::string xml = mdl::generate_mjcf(*req);
    const auto compiled = mdl::compile_mjcf(xml);
    trn::populate_heightfield_3d(*compiled, hfield_grid, hf);
    auto rt_cfg = std::make_unique<runtime::RuntimeConfig>(
        runtime::project_runtime(*req, *compiled, track));
    const auto img = init::capture_model_image(*compiled);
    auto writer_cfg = std::make_unique<nativecfg::WriterConfig>(
        nativecfg::decode(rt_cfg->writer_config));
    writer_cfg->tire.reset();
    writer_cfg->cruise.reset();
    writer_cfg->compliant_3d_tire = make_compliant_3d_tire_config(
        req->spatial->tire_preset, make_terrain3d_callbacks(terrain3d));
    if (writer_cfg->resistance.has_value()) {
        writer_cfg->resistance->allow_spatial = true;
    }

    const int root_free_id = mj_name2id(compiled.get(), mjOBJ_JOINT, "root_free");
    bt::require(root_free_id >= 0, "spatial model has root_free");
    const int qa = std::views::counted(
        compiled->jnt_qposadr, compiled->njnt)[static_cast<std::size_t>(root_free_id)];

    const auto check_yaw = [&](double yaw_deg, double expected_grade) {
        Stepper stepper(img.mjb, *writer_cfg);
        const mjModel *m = stepper.model();
        mjData *d = stepper.data();
        const double yaw_rad = yaw_deg * std::numbers::pi / 180.0;
        const sp::Quat q = sp::quat_from_euler_zyx(0.0, 0.0, yaw_rad);
        const auto qpos = std::views::counted(d->qpos, m->nq);
        qpos[static_cast<std::size_t>(qa) + 0U] = 20.0;
        qpos[static_cast<std::size_t>(qa) + 1U] = 0.0;
        qpos[static_cast<std::size_t>(qa) + 2U] = 0.15 * 20.0 + 0.5;
        qpos[static_cast<std::size_t>(qa) + 3U] = q[0];
        qpos[static_cast<std::size_t>(qa) + 4U] = q[1];
        qpos[static_cast<std::size_t>(qa) + 5U] = q[2];
        qpos[static_cast<std::size_t>(qa) + 6U] = q[3];
        mj_forward(m, d);

        auto phys = runtime::make_physical_step(stepper, *rt_cfg);
        bt::require(phys->has_launch_support(),
                    "spatial standstill PhysicalStep enables LaunchSupport");
        bt::require(phys->has_steer_balance(),
                    "articulated_3d PhysicalStep enables SteerBalance");
        const double got = phys->rider_senses().grade_ahead;
        bt::require(std::abs(got - expected_grade) <= 5e-3,
                    "3D heading lookahead grade matches directional slope at yaw " +
                        std::to_string(static_cast<int>(yaw_deg)) + " deg");

        bool threw = false;
        try {
            phys->set_direct_steer_torque_nm(1.0);
        } catch (const std::logic_error &) {
            threw = true;
        }
        bt::require(threw,
                    "articulated_3d rejects direct_steer_torque_nm override");
    };

    check_yaw(0.0, grade);
    check_yaw(45.0, grade * std::cos(std::numbers::pi / 4.0));
    check_yaw(90.0, 0.0);
}

[[gnu::noinline]] std::unique_ptr<init::SetupRequest>
build_articulated_3d_launch_request(double grade, double &release_speed_out) {
    auto req = std::make_unique<init::SetupRequest>();
    req->track = make_constant_grade_track(140.0, grade);
    req->start_x_m = 10.0;
    req->model.mode = "ride";
    req->model.crank_joint = true;
    req->model.rider = mdl::rider_from_args("articulated_3d");
    req->model.physics = *load_articulated_launch_physics();
    const sp::SpatialConfig spatial_cfg = make_standstill_spatial_config();
    req->spatial = spatial_cfg;
    req->model.field = trn::HeightFieldSpec::for_track(req->track);
    req->runtime.record_retention = runtime::RetentionMode::full;
    if (!spatial_cfg.start_support.has_value()) {
        throw std::runtime_error("standstill spatial config missing start_support");
    }
    release_speed_out = spatial_cfg.start_support->release_speed_mps;
    return req;
}

[[gnu::noinline]] void restore_bootstrap_into_stepper(
    Stepper &stepper, const runtime::BootstrapState &bs_state) {
    mjModel *m = stepper.model();
    mjData *d = stepper.data();
    stepper.mutate([&] {
        mj_resetData(m, d);
        const auto frictionloss = model_access::mutable_buffer(
            m->dof_frictionloss, m->nv, "dof_frictionloss");
        for (const auto &entry : bs_state.model_mutable.dof_frictionloss) {
            frictionloss[static_cast<std::size_t>(entry.dof_adr)] = entry.value;
        }
        const auto site_pos = model_access::mutable_buffer(
            m->site_pos, m->nsite * 3, "site_pos");
        for (const auto &entry : bs_state.model_mutable.site_pos) {
            const auto base = 3U * static_cast<std::size_t>(entry.site_id);
            for (std::size_t i = 0; i < 3U; ++i) {
                site_pos[base + i] = entry.value[i];
            }
        }
        mj_setConst(m, d);
        mj_setState(m, d, bs_state.integration_state.data(), mjSTATE_INTEGRATION);
        stepper.drive().restore(bs_state.drive);
        if (stepper.has_rider_contacts() &&
            !bike::io::is_null(bs_state.rider_contacts)) {
            auto [cs, pr] = rider::decode_rider_contacts_state(
                bs_state.rider_contacts, stepper.rider_contacts());
            stepper.rider_contacts().assign_state(std::move(cs), std::move(pr));
        }
        if (stepper.has_tire()) {
            stepper.set_tire_state(bs_state.tire.names, bs_state.tire.row);
        }
        mj_forward(m, d);
        stepper.note_posvel_fresh();
    });
}

[[gnu::noinline]] std::unique_ptr<runtime::PeriodAccounting> make_launch_accounting(
    const Stepper &stepper, runtime::PhysicalStep &phys_step,
    const runtime::RuntimeConfig &rt_cfg, std::size_t root_qa,
    std::size_t root_da) {
    const mjModel *m = stepper.model();
    const auto [e0, sc0] = phys_step.measure_energy_and_scale();
    runtime::AccountingConfig acc_cfg;
    acc_cfg.timestep_s = rt_cfg.timestep_s;
    acc_cfg.period_steps = static_cast<std::size_t>(rt_cfg.control_period_steps);
    acc_cfg.measurement_decimation = rt_cfg.measurement_decimation;
    acc_cfg.retention = runtime::RetentionMode::full;
    acc_cfg.max_qpos = static_cast<std::size_t>(m->nq);
    acc_cfg.max_dofs = static_cast<std::size_t>(m->nv);
    acc_cfg.columns = {.root_x_qpos = root_qa,
                       .root_x_dof = root_da,
                       .root_pitch_qpos = root_qa + 4U};
    auto acc_init = std::make_unique<runtime::AccountingState>();
    acc_init->has_launch_support = phys_step.has_launch_support();
    acc_init->energy.initial_energy_j = e0;
    acc_init->energy.energy_scale_j = sc0;
    acc_init->energy.mechanical_energy_j = e0;
    return std::make_unique<runtime::PeriodAccounting>(
        acc_cfg, *acc_init, 1,
        phys_step.rider_present() ? &phys_step.rider_control() : nullptr,
        stepper.has_rider_contacts() ? &stepper.rider_contacts() : nullptr);
}

[[gnu::noinline]] void run_articulated_3d_launch_scenario(double grade) {
    const std::string label =
        "articulated_3d_launch_grade_" + std::to_string(static_cast<int>(grade * 100.0)) + "pct";
    double release_speed_mps = 0.0;
    auto req = build_articulated_3d_launch_request(grade, release_speed_mps);
    if (!req->spatial.has_value()) {
        throw std::runtime_error(label + ": spatial config present");
    }
    const auto &spatial_cfg = *req->spatial;
    const double ground_z_m =
        bike::geometry::compute_ground_z(req->model.bike) / 1000.0;
    const auto terrain = std::make_unique<trn::ExtrudedTrack3d>(
        req->track, spatial_cfg.corridor_half_width_m, ground_z_m);
    const sp::SpatialTirePreset tire_preset = spatial_cfg.tire_preset;

    const auto owner = make_owner_from_ptr(std::move(req));
    const auto eq = init::settle(*owner);
    bt::require(std::isfinite(eq.residual_qacc) && eq.residual_qacc <= 0.1,
                label + ": SetupOwner::settle achieves static equilibrium");

    auto bs = std::make_unique<runtime::RuntimeBootstrap>(owner->seal_bootstrap());
    auto writer_cfg = std::make_unique<nativecfg::WriterConfig>(
        nativecfg::decode(bs->config.writer_config));
    writer_cfg->tire.reset();
    writer_cfg->cruise.reset();
    writer_cfg->compliant_3d_tire = make_compliant_3d_tire_config(
        tire_preset, make_terrain3d_callbacks(*terrain));
    if (writer_cfg->resistance.has_value()) {
        writer_cfg->resistance->allow_spatial = true;
    }

    auto stepper = std::make_unique<Stepper>(bs->model.mjb, *writer_cfg);
    writer_cfg.reset();
    restore_bootstrap_into_stepper(*stepper, bs->state);
    const mjModel *m = stepper->model();
    const mjData *d = stepper->data();

    auto phys_step = runtime::make_physical_step(*stepper, bs->config);
    {
        auto step_st = phys_step->state();
        step_st.initializing = false;
        step_st.generation = 1;
        phys_step->restore(step_st);
    }

    const int root_free_id = mj_name2id(m, mjOBJ_JOINT, "root_free");
    bt::require(root_free_id >= 0, label + ": compiled model has root_free");
    const auto jnt_qpos = std::views::counted(m->jnt_qposadr, m->njnt);
    const auto jnt_dof = std::views::counted(m->jnt_dofadr, m->njnt);
    const auto root_qa = static_cast<std::size_t>(
        jnt_qpos[static_cast<std::size_t>(root_free_id)]);
    const auto root_da = static_cast<std::size_t>(
        jnt_dof[static_cast<std::size_t>(root_free_id)]);

    auto accounting = make_launch_accounting(
        *stepper, *phys_step, bs->config, root_qa, root_da);

    runtime::RideControl control{};
    control.rider_enabled = true;
    check_probe_components(*phys_step, control, true, label);

    const std::int64_t total_steps =
        static_cast<std::int64_t>(std::nearbyint(5.0 / bs->config.timestep_s));
    const auto period_steps =
        static_cast<std::int64_t>(bs->config.control_period_steps);
    const std::vector<double> zero_ext(static_cast<std::size_t>(m->nv), 0.0);
    const auto qpos = std::views::counted(d->qpos, m->nq);
    const auto qvel = std::views::counted(d->qvel, m->nv);

    double max_abs_roll_rad = 0.0;
    double max_abs_y_m = 0.0;
    double max_speed_mps = 0.0;
    double max_hand_force_n = 0.0;
    bool observed_launch_support_work = false;
    double final_residual_ratio = 0.0;

    for (std::int64_t step_idx = 0; step_idx < total_steps; ++step_idx) {
        auto raw = phys_step->advance_physics(0.0, 0.0, zero_ext, control, nullptr);
        accounting->push(std::move(raw));
        if ((step_idx + 1) % period_steps == 0) {
            accounting->flush();
        }
        if (stepper->has_rider_contacts()) {
            const auto &diag = stepper->rider_contacts().state().diagnostics;
            if (diag.evaluated) {
                const auto hand_norm = [](const auto &gs) {
                    return std::hypot(gs.force_on_rider_n[0],
                                      gs.force_on_rider_n[1],
                                      gs.force_on_rider_n[2]);
                };
                max_hand_force_n = std::max(
                    {max_hand_force_n,
                     hand_norm(diag.grip_sides[0]),
                     hand_norm(diag.grip_sides[1])});
            }
        }
        const sp::Quat quat{
            qpos[root_qa + 3U], qpos[root_qa + 4U],
            qpos[root_qa + 5U], qpos[root_qa + 6U]};
        const sp::EulerZyx att = sp::euler_zyx(quat);
        bt::require(att.valid, label + ": finite attitude during climb");
        max_abs_roll_rad = std::max(max_abs_roll_rad, std::abs(att.roll_rad));
        max_abs_y_m = std::max(max_abs_y_m, std::abs(qpos[root_qa + 1U]));
        max_speed_mps = std::max(max_speed_mps, qvel[root_da]);
    }
    accounting->flush();

    const auto batch = accounting->prepare_samples();
    bt::require(!batch.samples.empty(), label + ": recorded intervals present");
    for (const auto &sample : batch.samples) {
        if (const auto *sw = runtime::sample_wire::find(sample->channels, "sensors")) {
            const auto &sobj = runtime::sample_wire::object(*sw);
            if (const auto *v_wire = runtime::sample_wire::find(sobj, "forward_speed_mps")) {
                max_speed_mps = std::max(
                    max_speed_mps, runtime::sample_wire::number(*v_wire));
            }
        }
        if (const auto *aw = runtime::sample_wire::find(sample->channels, "attachment_samples")) {
            const auto &aobj = runtime::sample_wire::object(*aw);
            for (const char *grip : {"grip_left", "grip_right"}) {
                if (const auto *gw = runtime::sample_wire::find(aobj, grip)) {
                    const auto &gobj = runtime::sample_wire::object(*gw);
                    const double fn = runtime::sample_wire::number(
                        runtime::sample_wire::required(gobj, "normal_n"));
                    const double ft = runtime::sample_wire::number(
                        runtime::sample_wire::required(gobj, "tangent_n"));
                    max_hand_force_n = std::max(max_hand_force_n, std::hypot(fn, ft));
                }
            }
        }
        if (const auto *ew = runtime::sample_wire::find(sample->channels, "energy")) {
            const auto &eobj = runtime::sample_wire::object(*ew);
            const auto rq = runtime::research_quality(eobj, 0.01);
            final_residual_ratio = rq.residual_ratio;
            bt::require(rq.acceptable,
                        label + ": per-interval |energy.residual_j|/scale <= 1%");
            const auto *lsw = runtime::sample_wire::find(eobj, "launch_support_work_j");
            const auto *lsl = runtime::sample_wire::find(eobj, "launch_support_loss_j");
            if (lsw != nullptr && lsl != nullptr) {
                observed_launch_support_work = true;
                bt::require(std::isfinite(runtime::sample_wire::number(*lsw)) &&
                                std::isfinite(runtime::sample_wire::number(*lsl)),
                            label + ": launch_support work/loss are finite");
            }
        }
    }

    // Confirm release past v_release: once past v_release, probe should not apply launch_support.
    if (max_speed_mps > release_speed_mps + 0.1) {
        check_probe_components(*phys_step, control, false, label);
    }

    const auto acc_wire = accounting->state_wire();
    const auto &history_wire = runtime::sample_wire::object(
        runtime::sample_wire::required(acc_wire, "history"));
    const auto &support_wire = runtime::sample_wire::object(
        runtime::sample_wire::required(history_wire, "rider_support"));
    bt::require(runtime::rider_support_ok(support_wire),
                label + ": rider_support_ok passes (sit-bones & feet >= -6 N)");
    bt::require(max_hand_force_n <= 300.0,
                label + ": hand grip force <= 300 N");
    bt::require(max_abs_roll_rad < 0.35,
                label + ": launches without capsize (|roll| < 0.35 rad)");
    bt::require(max_abs_y_m < 1.5,
                label + ": stays in corridor (|y| < 1.5 m)");
    bt::require(max_speed_mps > release_speed_mps,
                label + ": launches past v_release");
    bt::require(observed_launch_support_work,
                label + ": energy wire records launch_support_work_j");
    bt::require(final_residual_ratio <= 0.01,
                label + ": final energy closure <= 1%");
}

[[gnu::noinline]] void test_articulated_3d_standstill_launch_climbs() {
    for (const double grade : {0.0, 0.15, 0.20}) {
        run_articulated_3d_launch_scenario(grade);
    }
}

} // namespace

int main() {
    // cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch
    return bt::run([] {
        test_launch_support_orthogonality_and_fade();
        test_planar_0bit_shield();
        test_3d_heading_lookahead_and_steer_guard();
        test_articulated_3d_standstill_launch_climbs();
    });
}
