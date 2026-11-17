// BGTRAJ01 external trajectory reader and `bike-replay --view --trajectory` CLI contracts.
#include "recording/recording_reader_test_support.hpp"
#include "recording/recording_fixture.hpp"
#include "engine_call.hpp"
#include "io/hash.hpp"
#include "platform/argv.hpp"
#include "platform/files.hpp"
#include "platform/process.hpp"
#include "platform/provenance.hpp"
#include "recording/reader.hpp"
#include "replay/playback.hpp"
#include "replay/trajectory.hpp"
#include "research/environment.hpp"
#include "research/evidence.hpp"
#include "runtime/control.hpp"
#include "runtime/research.hpp"
#include "terrain/profile.hpp"
#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace {
namespace io = bike::io;
namespace platform = bike::platform;
namespace rec = bike::recording;
namespace replay = bike::replay;
namespace research = bike::research;
using namespace rec::detail;

void append_bytes(std::vector<std::byte>& out, std::string_view text) {
    const auto raw = std::as_bytes(std::span{text});
    out.insert(out.end(), raw.begin(), raw.end());
}

void append_u32_le(std::vector<std::byte>& out, std::uint32_t value) {
    for (unsigned i = 0; i < 4; ++i) {
        out.push_back(static_cast<std::byte>((value >> (8U * i)) & 0xffU));
    }
}

void append_u64_le(std::vector<std::byte>& out, std::uint64_t value) {
    for (unsigned i = 0; i < 8; ++i) {
        out.push_back(static_cast<std::byte>((value >> (8U * i)) & 0xffU));
    }
}

void append_f64_le(std::vector<std::byte>& out, double value) {
    append_u64_le(out, std::bit_cast<std::uint64_t>(value));
}

struct TrajectorySpec {
    std::string magic = "BGTRAJ01";
    std::uint32_t version = 1;
    std::uint32_t wheel_cols = 8;
    std::uint32_t nq_override = 0;
    std::uint32_t nv_override = 0;
    std::size_t drop_bytes = 0;
    std::string sha_override;
};

void write_trajectory_dir(
    const std::filesystem::path& dir,
    const runtime::ModelImage& image,
    const replay::PlaybackData& src,
    std::size_t frames,
    const TrajectorySpec& spec) {
    std::filesystem::create_directories(dir);
    platform::write_exclusive(dir / "model.mjb", image.mjb);

    const auto nq = spec.nq_override != 0
        ? spec.nq_override
        : static_cast<std::uint32_t>(src.qpos.front().size());
    const auto nv = spec.nv_override != 0
        ? spec.nv_override
        : static_cast<std::uint32_t>(src.qvel.front().size());

    std::vector<std::byte> bytes;
    append_bytes(bytes, spec.magic);
    append_u32_le(bytes, spec.version);
    append_u32_le(bytes, nq);
    append_u32_le(bytes, nv);
    append_u32_le(bytes, spec.wheel_cols);
    append_u64_le(bytes, static_cast<std::uint64_t>(frames));

    std::string sha = spec.sha_override.empty() ? image.sha256 : spec.sha_override;
    sha.resize(64, '0');
    append_bytes(bytes, sha);

    const std::size_t row_nq = src.qpos.front().size();
    const std::size_t row_nv = src.qvel.front().size();
    for (std::size_t f = 0; f < frames; ++f) {
        append_f64_le(bytes, src.times_s[f]);
        append_f64_le(bytes, src.ends_s[f]);
        for (std::uint32_t i = 0; i < nq; ++i) {
            append_f64_le(bytes, i < row_nq ? src.qpos[f][i] : 0.0);
        }
        for (std::uint32_t i = 0; i < nv; ++i) {
            append_f64_le(bytes, i < row_nv ? src.qvel[f][i] : 0.0);
        }
        for (std::uint32_t w = 0; w < spec.wheel_cols; ++w) {
            append_f64_le(bytes, static_cast<double>(w));
        }
    }
    if (spec.drop_bytes > 0 && spec.drop_bytes <= bytes.size()) {
        bytes.resize(bytes.size() - spec.drop_bytes);
    }
    platform::write_exclusive(dir / "trajectory.bin", bytes);
}

std::unique_ptr<research::ResearchRequest> roundtrip_request() {
    auto owner = std::make_unique<research::ResearchRequest>();
    auto& request = *owner;
    bike::configuration::SimulationPhysicsConfig physics;
    physics.physics_mode = "physical";
    physics.drive_mode = "crank_effort";
    physics.drive.motor_clutch = true;
    physics.timestep_s = .0005;
    physics.control_period_s = .005;
    physics.monitors.grounded_hold_s = .005;
    physics.monitors.contact_dropout_s = 10. * physics.timestep_s;
    physics.initial_speed_mps = .1;
    physics.equilibrium_cache_enabled = false;
    physics.tires.backend = "compliant_2d";
    physics.tires.surface_mode = "track";
    physics.seated_climb.period_s = .00125;
    physics.seated_climb.reaction_delay_s = .15;
    physics.seated_climb.lean_rate_rad_s = .5;
    physics.seated_climb.orientation_tau_s = .5;
    physics.seated_climb.surge_budget_s = 15.;
    physics.seated_climb.trim_dead_time_s = .2;
    physics.seated_climb.max_crank_torque_nm = 60.;
    physics.seated_climb.max_forward_lean_rad = .55;
    physics.seated_climb.min_elbow_flexion_deg = 15.0;
    physics.seated_climb.climb_effort_cap_grades = {0.};
    physics.seated_climb.climb_effort_cap_grade_nm = {1000.};
    physics.seated_climb.climb_effort_rise_speed_mps = 2.5;
    physics.seated_climb.climb_effort_full_speed_mps = 4.0;
    physics.rider_launch = {
        .start_cog_grade_breaks = {},
        .start_cog_teeth = {24},
        .start_pedal_angle_rad = 0.,
        .start_lean_grades = {0.},
        .start_lean_rad = {0.},
        .plan_enabled = false,
        .standstill_speed_mps = .1,
        .standstill_crank_rpm = 5.,
        .effort_rate_nm_s = 40.,
        .effort_cap_asphalt_nm = 60.,
        .effort_cap_hardpack_nm = 60.,
        .effort_cap_loose_nm = 60.,
        .effort_cap_wet_nm = 60.,
        .effort_cap_scree_nm = 60.,
        .effort_cap_grades = {0.},
        .effort_cap_grade_nm = {1000.},
        .effort_rise_speed_mps = 2.5,
        .release_torque_grades = {0., .2},
        .release_torque_nm = {0., 30.},
        .brake_release_s = .15,
        .max_hold_s = 1.5,
        .upshift_crank_rpm = 82.,
        .shift_retry_s = .25,
        .end_speed_mps = 3.,
        .end_crank_rpm = 60.,
        .end_hold_s = .3,
        .max_launch_s = 6.,
        .handover_s = .9,
    };
    physics.front_lift_reaction = {
        .hand_window_s = .5,
        .hand_lag_s = .5,
        .hand_drop_ratio = .7,
        .pitch_rate_up_rad_s = .5,
        .pitch_rad = .17,
        .decision_s = .1,
        .ease_enabled = false,
        .lean_enabled = false,
        .rear_brake_tap_enabled = false,
        .ease_fraction = .7,
        .ease_rate_per_s = 5.,
        .lean_add_rad = .1,
        .lean_rate_rad_s = .5,
        .clear_s = .3,
    };
    physics.drive.assist.release_hold_s = .3;
    physics.drive.pedaling.stop_time_s = .35;
    physics.drive.pedaling.reposition_on_stall = false;
    physics.drive.pedaling.reposition_min_effort_nm = 20.;
    physics.drive.pedaling.reposition_stall_window_s = 1.;
    physics.drive.pedaling.reposition_stall_progress_rad = .5;
    physics.drive.pedaling.reposition_cooldown_s = 1.;
    physics.drive.pedaling.reposition_back_rate_rad_s = 3.;
    physics.drive.pedaling.reposition_damping_nms_rad = 12.;
    physics.drive.pedaling.reposition_timeout_s = 3.;
    physics.drive.pedaling.reposition_phase_tolerance_rad = .05;
    physics.drive.pedaling.reposition_noop_rad = .12;
    physics.drive.shifting.shift_cooldown_s = .4;
    physics.drive.shifting.shift_cut_duration_s = .2;
    physics.articulated.balance_dwell_s = .5;
    physics.articulated.balance_grace_s = 3.;
    physics.articulated.joint_kp_nm_rad = 600.;
    physics.articulated.joint_kd_nms_rad = 15.;
    physics.articulated.leg_active_damping_nms_rad = 35.;
    physics.equilibrium_crank_hold_kp_nm_rad = 20000.;
    physics.equilibrium_crank_hold_kd_nms_rad = 88.;
    physics.research.control_period_s = .01;
    physics.research.actuator_delay_s = .005;
    physics.research.sensor_latency_s = .01;
    physics.research.sensor_maximum_age_s = .1;
    physics.research.sample_period_s = .005;
    physics.research.record_decimation = 80;
    bike::configuration::RiderSpecs rider;
    rider.variant = "lumped";
    request.setup.model.physics = physics;
    request.setup.model.rider = rider;
    request.setup.model.include_rider = true;
    request.setup.model.mode = "ride";
    request.setup.model.crank_joint = true;
    request.setup.track = bike::terrain::flat(12.);
    request.experiment.duration_s = .006;
    request.experiment.control_period_s = .002;
    request.experiment.actuator_delay_s = .001;
    request.experiment.record_decimation = 1;
    request.experiment.stop_on_model_violation = false;
    request.sensors = research::SensorConfig::ideal();
    request.sensors.sample_period_s = .001;
    request.sensors.delivery.maximum_age_s = .1;
    request.road_resolution_m = .05;
    request.capture_mode = runtime::ResearchCaptureMode::full;
    return owner;
}

rec::Publication record_sealed(const std::filesystem::path& root) {
    auto prepared = research::prepare_environment(roundtrip_request());
    bike::model::ModelOwner const model{engine::load_model_image(prepared->bootstrap.ride.model.mjb)};
    const auto catalog = rec::build_catalog(*model);
    auto environment = runtime::make_research(
        std::make_unique<runtime::ResearchBootstrap>(std::move(prepared->bootstrap)));
    runtime::RideControl command;
    command.motor_torque_nm = 0.;
    command.human_torque_nm = 0.;
    for (;;) {
        const auto status = research::wire_value(runtime::Wire{environment->status()});
        if (io::as_bool(field(status, "terminated"), "terminated") ||
            io::as_bool(field(status, "truncated"), "truncated")) {
            break;
        }
        environment->begin_control(command, 0., 0.);
        bt::require(environment->advance_control({}).has_value(),
                    "unbounded native control completes a physical window");
        environment->acknowledge_control();
    }
    auto owner = environment->seal_episode();
    set(owner.run_metadata, "identity_checks", research::identity_receipt(owner.configuration.metadata));
    const auto summary = research::summarize(owner);
    rec::RunCapture run(recording_fixture::metadata("00000000-0000-4000-8000-000000000099"));
    run.add_episode(rec::make_research_evidence(0, std::move(owner), catalog, summary));
    auto publication = rec::finalize(run, root / "runs");
    environment->close();
    return publication;
}

template <class Action>
void require_recording_error(Action&& action, std::string_view needle) {
    bool threw = false;
    std::string message;
    try {
        std::forward<Action>(action)();
    } catch (const rec::RecordingError& error) {
        threw = true;
        message = error.what();
    }
    bt::require(threw, "expected RecordingError");
    bt::require(message.find(needle) != std::string::npos,
                "expected '" + std::string(needle) + "' in '" + message + "'");
}

void loader_contracts(const rec::Publication& pub, const replay::PlaybackData& src) {
    recording_reader_test::Directory const tmp;
    const std::size_t frames = std::min<std::size_t>(3, src.times_s.size());
    bt::require(frames >= 2, "recorded source has at least two frames");

    const auto valid_dir = tmp.path / "valid";
    write_trajectory_dir(valid_dir, src.model, src, frames, TrajectorySpec{});
    const auto ext = replay::load_external_trajectory(valid_dir);
    bt::require(ext.playback.times_s.size() == frames && ext.wheels.size() == frames, "frame counts match");
    bt::require(ext.playback.duration_s == src.ends_s[frames - 1], "duration is the last interval end");
    bt::require(ext.playback.qpos[1] == src.qpos[1], "qpos rows round-trip exactly");
    bt::require(ext.playback.qvel[1] == src.qvel[1], "qvel rows round-trip exactly");
    bt::require(ext.wheels[0][7] == 7.0, "wheel columns round-trip");
    bt::require(!ext.playback.intervals_hash_verified, "external trajectory is marked unverified/presentation");
    const replay::PlaybackSession session(ext.playback);
    bt::require(session.duration_s() == ext.playback.duration_s, "PlaybackSession accepts external PlaybackData");

    {
        TrajectorySpec s;
        s.sha_override = std::string(64, 'a');
        write_trajectory_dir(tmp.path / "bad_hash", src.model, src, 2, s);
    }
    require_recording_error([&] { (void)replay::load_external_trajectory(tmp.path / "bad_hash"); },
                            "trajectory model hash");

    {
        TrajectorySpec s;
        s.drop_bytes = 8;
        write_trajectory_dir(tmp.path / "truncated", src.model, src, 2, s);
    }
    require_recording_error([&] { (void)replay::load_external_trajectory(tmp.path / "truncated"); },
                            "trajectory size");

    {
        TrajectorySpec s;
        s.magic = "BADMAGIC";
        write_trajectory_dir(tmp.path / "bad_magic", src.model, src, 2, s);
    }
    require_recording_error([&] { (void)replay::load_external_trajectory(tmp.path / "bad_magic"); },
                            "trajectory magic");

    {
        TrajectorySpec s;
        s.version = 99;
        write_trajectory_dir(tmp.path / "bad_ver", src.model, src, 2, s);
    }
    require_recording_error([&] { (void)replay::load_external_trajectory(tmp.path / "bad_ver"); },
                            "trajectory version");

    {
        TrajectorySpec s;
        s.wheel_cols = 4;
        write_trajectory_dir(tmp.path / "bad_wcols", src.model, src, 2, s);
    }
    require_recording_error([&] { (void)replay::load_external_trajectory(tmp.path / "bad_wcols"); },
                            "wheel_cols");

    {
        TrajectorySpec s;
        s.nq_override = static_cast<std::uint32_t>(src.qpos.front().size() + 1);
        write_trajectory_dir(tmp.path / "bad_nq", src.model, src, 2, s);
    }
    require_recording_error([&] { (void)replay::load_external_trajectory(tmp.path / "bad_nq"); },
                            "dimensions differ");

    write_trajectory_dir(tmp.path / "one_frame", src.model, src, 1, TrajectorySpec{});
    require_recording_error([&] { (void)replay::load_external_trajectory(tmp.path / "one_frame"); },
                            "at least two");

    // Verify CLI dry-run and rejections against the same sealed recording.
    (void)pub;
}

void cli_dry_run_contracts(const std::filesystem::path& replay_bin,
                           const rec::Publication& pub,
                           const replay::PlaybackData& src) {
    recording_reader_test::Directory const tmp;
    const std::size_t frames = std::min<std::size_t>(3, src.times_s.size());
    const auto valid_dir = tmp.path / "valid";
    write_trajectory_dir(valid_dir, src.model, src, frames, TrajectorySpec{});

    platform::ProcessOptions opts;
    opts.timeout_s = 120.;

    const auto ok = platform::run(
        {replay_bin.string(), pub.directory.string(), "--view", "--trajectory", valid_dir.string(), "--dry-run"},
        tmp.path, opts);
    bt::require(ok.exit_code == 0 && ok.out.find("frames=") != std::string::npos &&
                    ok.out.find("ok") != std::string::npos,
                "--view --trajectory --dry-run succeeds: out=" + ok.out + " err=" + ok.err);

    const auto no_view = platform::run(
        {replay_bin.string(), pub.directory.string(), "--trajectory", valid_dir.string()},
        tmp.path, opts);
    bt::require(no_view.exit_code == 2 && no_view.err.find("--trajectory requires --view") != std::string::npos,
                "--trajectory without --view fails with exit 2");

    const auto with_viewer = platform::run(
        {replay_bin.string(), pub.directory.string(), "--viewer", "--trajectory", valid_dir.string()},
        tmp.path, opts);
    bt::require(with_viewer.exit_code == 2, "--viewer with --trajectory fails with exit 2");

    {
        TrajectorySpec s;
        s.magic = "BADMAGIC";
        write_trajectory_dir(tmp.path / "bad_magic", src.model, src, 2, s);
    }
    const auto bad_magic = platform::run(
        {replay_bin.string(), pub.directory.string(), "--view", "--trajectory",
         (tmp.path / "bad_magic").string(), "--dry-run"},
        tmp.path, opts);
    bt::require(bad_magic.exit_code == 1 && bad_magic.err.find("REPLAY ERROR") != std::string::npos,
                "bad-magic trajectory fails with exit 1");

    {
        TrajectorySpec s;
        s.drop_bytes = 8;
        write_trajectory_dir(tmp.path / "truncated", src.model, src, 2, s);
    }
    const auto truncated = platform::run(
        {replay_bin.string(), pub.directory.string(), "--view", "--trajectory",
         (tmp.path / "truncated").string(), "--dry-run"},
        tmp.path, opts);
    bt::require(truncated.exit_code == 1 && truncated.err.find("REPLAY ERROR") != std::string::npos,
                "truncated trajectory fails with exit 1");

    {
        TrajectorySpec s;
        s.nq_override = static_cast<std::uint32_t>(src.qpos.front().size() + 1);
        write_trajectory_dir(tmp.path / "bad_nq", src.model, src, 2, s);
    }
    const auto bad_nq = platform::run(
        {replay_bin.string(), pub.directory.string(), "--view", "--trajectory",
         (tmp.path / "bad_nq").string(), "--dry-run"},
        tmp.path, opts);
    bt::require(bad_nq.exit_code == 1 && bad_nq.err.find("REPLAY ERROR") != std::string::npos,
                "nq-mismatch trajectory fails with exit 1");

    // Mutate one byte of model.mjb so the trajectory directory is internally
    // consistent (matching its own model.mjb) yet differs from the recording's model.
    auto altered_model = src.model;
    altered_model.mjb.back() = static_cast<std::byte>(std::to_integer<unsigned char>(altered_model.mjb.back()) ^ 0x01U);
    altered_model.sha256 = io::sha256(altered_model.mjb);
    write_trajectory_dir(tmp.path / "diff_model", altered_model, src, 2, TrajectorySpec{});
    const auto diff_model = platform::run(
        {replay_bin.string(), pub.directory.string(), "--view", "--trajectory",
         (tmp.path / "diff_model").string(), "--dry-run"},
        tmp.path, opts);
    bt::require(diff_model.exit_code == 1 &&
                    diff_model.err.find("--trajectory model differs") != std::string::npos,
                "mismatched trajectory model fails with exit 1");
}

std::filesystem::path resolve_replay_binary(int argc, char** argv) {
    if (argc >= 2) {
        const auto args = platform::main_arguments(argc, argv);
        return {args[1]};
    }
    return platform::executable_path().parent_path().parent_path().parent_path().parent_path() / "bin" / "bike-replay";
}
} // namespace

int main(int argc, char** argv) {
    return bt::run([&] {
        const auto replay_bin = resolve_replay_binary(argc, argv);
        bt::require(std::filesystem::exists(replay_bin), "bike-replay binary exists: " + replay_bin.string());
        recording_reader_test::Directory const tmp;
        const auto pub = record_sealed(tmp.path);
        const auto src = replay::load_playback(pub.directory, 0);
        loader_contracts(pub, src);
        cli_dry_run_contracts(replay_bin, pub, src);
    });
}
