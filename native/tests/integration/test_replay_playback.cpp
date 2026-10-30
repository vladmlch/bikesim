// Port of tests/test_replay_playback.py — the stored-state playback viewer:
// load a recorded episode's frames, interpolate poses (linear dofs exact,
// quaternion blocks renormalized), refuse missing/empty inputs, and keep
// `bike-replay --view --dry-run` presentation-only while the default path
// still verifies.
#include "../reference/replay_reference_support.hpp"
#include "platform/process.hpp"
#include "platform/provenance.hpp"

namespace {
namespace rr = replay_reference;
namespace replay = bike::replay;

[[nodiscard]] bool all_finite(std::span<const double> values) {
    return std::ranges::all_of(values, [](double value) { return std::isfinite(value); });
}

// test_playback_loads_episode — one real recorded container: strictly
// increasing finite times, (n,nq)/(n,nv) frames, positive finite duration,
// poses at the endpoints and midpoint, and exact endpoint frames.
void loads_episode() {
    rr::Directory tmp;
    const auto publication = rr::record_container(tmp.path);
    const auto data = replay::load_playback(publication.directory, 0);
    const auto model = replay::make_playback_model(data);
    const auto nq = static_cast<std::size_t>(model->nq);
    const auto nv = static_cast<std::size_t>(model->nv);
    bt::require(!data.times_s.empty() && all_finite(data.times_s),
        "times_s is a finite nonempty sequence");
    for (std::size_t i = 1; i < data.times_s.size(); ++i)
        bt::require(data.times_s[i] > data.times_s[i - 1], "times_s is strictly increasing");
    const auto n = data.times_s.size();
    bt::require(data.qpos.size() == n && data.qvel.size() == n,
        "recorded states match the frame count");
    for (const auto& row : data.qpos) bt::require(row.size() == nq, "qpos rows match model nq");
    for (const auto& row : data.qvel) bt::require(row.size() == nv, "qvel rows match model nv");
    bt::require(std::isfinite(data.duration_s) && data.duration_s > 0.,
        "duration_s is positive and finite");
    for (const double t : {0., .5 * data.duration_s, data.duration_s}) {
        const auto pose = replay::pose_at(data, t);
        bt::require(pose.size() == nq && all_finite(pose),
            "pose_at returns a finite nq pose inside the interval");
    }
    bt::require(std::ranges::equal(replay::pose_at(data, data.times_s.front()), data.qpos.front()) &&
        std::ranges::equal(replay::pose_at(data, data.times_s.back()), data.qpos.back()),
        "recorded frame times replay their stored poses exactly");
}

// test_playback_pose_interpolates — midpoints land on the linear mean for
// non-quaternion dofs; quaternion blocks stay unit norm.
void pose_interpolates() {
    rr::Directory tmp;
    const auto publication = rr::record_container(tmp.path);
    const auto data = replay::load_playback(publication.directory, 0);
    const auto model = replay::make_playback_model(data);
    const auto nq = static_cast<std::size_t>(model->nq);
    bt::require(data.times_s.size() >= 2,
        "the episode records at least two poses to interpolate between");
    std::vector<bool> quaternion(nq, false);
    for (const auto start : data.quaternion_starts)
        for (std::size_t i = start; i < start + 4; ++i) quaternion.at(i) = true;
    for (std::size_t k = 0; k + 1 < data.times_s.size(); ++k) {
        const auto pose = replay::pose_at(data, .5 * (data.times_s[k] + data.times_s[k + 1]));
        for (std::size_t i = 0; i < nq; ++i) {
            if (quaternion[i]) continue;
            bt::require(std::abs(pose[i] - .5 * (data.qpos[k][i] + data.qpos[k + 1][i])) <= 1e-12,
                "non-quaternion dofs land on the exact midpoint");
        }
        for (const auto start : data.quaternion_starts) {
            double norm = 0.;
            for (std::size_t i = start; i < start + 4; ++i) norm = std::hypot(norm, pose[i]);
            bt::require(std::abs(norm - 1.) < 1e-9,
                "each quaternion block stays normalized at the midpoint");
        }
    }
}

// test_playback_rejects_bad_dir — missing or frame-less directories raise,
// never hang or silently succeed.
void rejects_bad_dirs() {
    rr::Directory tmp;
    rr::rejects([&] { (void)replay::load_playback(tmp.path); });
    rr::rejects([&] { (void)replay::load_playback(tmp.path / "nonexistent"); });
    const auto fake = tmp.path / "fake";
    std::filesystem::create_directory(fake);
    rr::write_file(fake / "summary.json", "{\"name\": \"not-an-episode\"}\n");
    rr::rejects([&] { (void)replay::load_playback(fake); });
}

[[nodiscard]] std::filesystem::path replay_binary() {
    // bike-replay lands at <build-root>/bin; this test's binary is four
    // levels down in native/tests/reference.
    return bike::platform::executable_path()
        .parent_path().parent_path().parent_path().parent_path() / "bin" / "bike-replay";
}

// test_replay_view_dry_run — --view --dry-run walks the stored trajectory
// without a window and exits 0; a missing recording reports 1.
void view_dry_run() {
    rr::Directory tmp;
    const auto publication = rr::record_container(tmp.path);
    const auto binary = replay_binary();
    bt::require(std::filesystem::exists(binary), "the bike-replay artifact is built");
    bike::platform::ProcessOptions options;
    options.timeout_s = 300.;
    const auto ok = bike::platform::run({binary.string(), publication.directory.string(),
        "--view", "--dry-run"}, tmp.path, options);
    bt::require(ok.exit_code == 0 && ok.stdout_text.find("frames=") != std::string::npos,
        "dry-run inspection walks the stored trajectory and reports 0");
    options.timeout_s = 60.;
    const auto missing = bike::platform::run({binary.string(),
        (tmp.path / "nonexistent").string(), "--view", "--dry-run"}, tmp.path, options);
    bt::require(missing.exit_code == 1 && missing.stderr_text.find("REPLAY ERROR") != std::string::npos,
        "a missing recording reports 1 on the dry-run path");
}

// test_replay_verify_still_default — without --view the CLI still verifies:
// bare and bad invocations fail nonzero, and a sealed replayable container
// verifies to exit 0.
void verify_still_default() {
    rr::Directory tmp;
    const auto binary = replay_binary();
    bt::require(std::filesystem::exists(binary), "the bike-replay artifact is built");
    bike::platform::ProcessOptions options;
    options.timeout_s = 30.;
    const auto bare = bike::platform::run({binary.string()}, tmp.path, options);
    bt::require(bare.exit_code != 0, "a missing positional argument fails nonzero");
    const auto missing = bike::platform::run({binary.string(),
        (tmp.path / "nonexistent").string()}, tmp.path, options);
    bt::require(missing.exit_code != 0, "a missing recording fails nonzero");
    const auto publication = rr::record_container(tmp.path);
    options.timeout_s = 300.;
    const auto verified = bike::platform::run({binary.string(),
        publication.directory.string()}, tmp.path, options);
    bt::require(verified.exit_code == 0,
        "the default checked path verifies a sealed container to exit 0");
}
} // namespace
int main() {
    return bt::run([] {
        loads_episode();
        pose_interpolates();
        rejects_bad_dirs();
        view_dry_run();
        verify_still_default();
    });
}
